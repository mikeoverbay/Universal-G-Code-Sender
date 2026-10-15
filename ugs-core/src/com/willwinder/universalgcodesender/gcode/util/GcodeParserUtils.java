/*
    Copyright 2017-2024 Will Winder

    This file is part of Universal Gcode Sender (UGS).

    UGS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    UGS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with UGS.  If not, see <http://www.gnu.org/licenses/>.
 */
package com.willwinder.universalgcodesender.gcode.util;

import com.google.common.collect.Iterables;
import com.willwinder.universalgcodesender.gcode.DefaultCommandCreator;
import com.willwinder.universalgcodesender.gcode.GcodeParser;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import static com.willwinder.universalgcodesender.gcode.util.Code.G20;
import static com.willwinder.universalgcodesender.gcode.util.Code.G21;
import static com.willwinder.universalgcodesender.gcode.util.Code.G53;
import static com.willwinder.universalgcodesender.gcode.util.Code.G90;
import static com.willwinder.universalgcodesender.gcode.util.Code.G90_1;
import static com.willwinder.universalgcodesender.gcode.util.Code.G91;
import static com.willwinder.universalgcodesender.gcode.util.Code.G91_1;
import static com.willwinder.universalgcodesender.gcode.util.Code.ModalGroup.Motion;
import static com.willwinder.universalgcodesender.gcode.util.Code.UNKNOWN;
import com.willwinder.universalgcodesender.i18n.Localization;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.types.GcodeCommand;
import com.willwinder.universalgcodesender.types.PointSegment;
import com.willwinder.universalgcodesender.utils.GcodeStreamReader;
import com.willwinder.universalgcodesender.utils.IGcodeStreamReader;
import com.willwinder.universalgcodesender.utils.IGcodeWriter;
import org.apache.commons.lang3.StringUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Collectors;

/**
 * @author wwinder
 */
public class GcodeParserUtils {
    private static final Logger LOGGER = Logger.getLogger(GcodeParserUtils.class.getName());
    private static final Set<Axis> XYZ = Collections.unmodifiableSet(EnumSet.of(Axis.X, Axis.Y, Axis.Z));

    /**
     * For backwards compatibility this method calls processCommand with includeNonMotionStates = false.
     */
    public static List<GcodeParser.GcodeMeta> processCommand(String command, int line, final GcodeState inputState)
            throws GcodeParserException {
        return processCommand(command, line, inputState, false);
    }

    /**
     * Process command given an initial state. This method will not modify its
     * input parameters.
     *
     * @param includeNonMotionStates Create gcode meta responses even if there is no motion, for example "F100" will not
     *                               return a GcodeMeta entry unless this flag is set to true.
     */
    public static List<GcodeParser.GcodeMeta> processCommand(String command, int line, final GcodeState inputState,
                                                             boolean includeNonMotionStates)
            throws GcodeParserException {
        List<String> args = GcodePreprocessorUtils.splitCommand(command);
        if (args.isEmpty()) return null;

        // Initialize with original state
        GcodeState state = inputState.copy();

        state.commandNumber = line;

        // handle M codes.
        Set<Code> mCodes = GcodePreprocessorUtils.getMCodes(args);
        for (Code c : mCodes) {
            switch (c.getType()) {
                case Spindle:
                    state.spindle = c;
                    break;
                case Coolant:
                    state.coolant = c;
                    break;
                default:
                    break;
            }
        }

        List<String> fCodes = GcodePreprocessorUtils.parseCodes(args, 'F');
        if (!fCodes.isEmpty()) {
            try {
                state.feedRate = Double.parseDouble(Iterables.getOnlyElement(fCodes));
            } catch (IllegalArgumentException e) {
                throw new GcodeParserException("Multiple F-codes on one line.");
            }
        }

        List<String> sCodes = GcodePreprocessorUtils.parseCodes(args, 'S');
        if (!sCodes.isEmpty()) {
            try {
                state.spindleSpeed = Double.parseDouble(Iterables.getOnlyElement(sCodes));
            } catch (IllegalArgumentException e) {
                throw new GcodeParserException("Multiple S-codes on one line.");
            }
        }

        // Gather G codes.
        Set<Code> gCodes = GcodePreprocessorUtils.getGCodes(args);

        boolean hasAxisWords = GcodePreprocessorUtils.hasAxisWords(args);

        // Error to mix group 1 (Motion) and certain group 0 (NonModal) codes (G10, G28, G30, G92)
        Collection<Code> motionCodes = gCodes.stream()
                .filter(Code::consumesMotion)
                .collect(Collectors.toList());

        // 1 motion code per line.
        if (motionCodes.size() > 1) {
            throw new GcodeParserException(Localization.getString("parser.gcode.multiple-axis-commands")
                    + ": " + StringUtils.join(motionCodes, ", "));
        }

        // If there are axis words and nothing to use them, add the currentMotionMode.
        if (hasAxisWords && motionCodes.isEmpty() && state.currentMotionMode != null) {
            gCodes.add(state.currentMotionMode);
        }

        // With G53 the axis words of the G0/G1 on this line are machine coordinates.
        boolean machineCoordinates = gCodes.contains(G53);

        // A tool change runs before the motion on its line and can leave the spindle anywhere.
        if (hasMCode(args, 6)) {
            state.unknownAxes.addAll(XYZ);
        }

        // Apply modal settings (units, plane, distance mode, G53, ...) before the codes that use the axis words,
        // whatever the order of the words on the line: "G1 X5 G91" is a relative move. The sort is stable.
        List<Code> orderedCodes = new ArrayList<>(gCodes);
        orderedCodes.sort(Comparator.comparing(Code::consumesMotion));

        // Apply each code to the state.
        List<GcodeParser.GcodeMeta> results = new ArrayList<>();
        for (Code i : orderedCodes) {
            if (i == UNKNOWN) {
                LOGGER.warning("An unknown gcode command was detected in: " + command);
            } else {
                GcodeParser.GcodeMeta meta = handleGCode(i, args, line, state, machineCoordinates);
                meta.command = command;
                // Commands like 'G21' don't return a point segment.
                if (meta.point != null) {
                    meta.point.setFeedRate(state.feedRate);
                    meta.point.setSpindleSpeed(state.spindleSpeed);
                }
                results.add(meta);
            }
        }

        // A program pause runs after the motion on its line. The operator may jog before resuming.
        if (mCodes.contains(Code.M0) || mCodes.contains(Code.M1) || mCodes.contains(Code.M60)) {
            state.unknownAxes.addAll(XYZ);
            results.forEach(meta -> meta.state.unknownAxes.addAll(XYZ));
        }

        // Return updated state / command.
        if (results.isEmpty() && includeNonMotionStates) {
            GcodeParser.GcodeMeta meta = new GcodeParser.GcodeMeta();
            meta.state = state;
            meta.command = command;
            meta.code = state.currentMotionMode;
            return Collections.singletonList(meta);
        }

        return results;
    }

    private static PointSegment addProbePointSegment(Position nextPoint, boolean fastTraverse, int line, GcodeState state) {
        if (nextPoint == null) {
            return null;
        }

        PointSegment ps = addLinearPointSegment(nextPoint, fastTraverse, line, state);
        ps.setIsProbe(true);
        return ps;
    }

    /**
     * Create a PointSegment representing the linear command.
     */
    private static PointSegment addLinearPointSegment(Position nextPoint, boolean fastTraverse, int line, GcodeState state) {
        if (nextPoint == null) {
            return null;
        }

        PointSegment ps = new PointSegment(nextPoint, line);

        boolean zOnly = state.currentPoint.isZMotionTo(nextPoint);
        boolean isRotation = state.currentPoint.hasRotationTo(nextPoint);

        ps.setIsMetric(state.isMetric);
        ps.setIsZMovement(zOnly);
        ps.setIsRotation(isRotation);
        ps.setIsFastTraverse(fastTraverse);

        // Save off the endpoint.
        state.currentPoint = nextPoint;
        return ps;
    }

    /**
     * Create a PointSegment representing the arc command.
     */
    private static PointSegment addArcPointSegment(Position nextPoint, boolean clockwise, List<String> args, int line, GcodeState state) {
        if (nextPoint == null) {
            return null;
        }

        PointSegment ps = new PointSegment(nextPoint, line);

        PlaneFormatter plane = new PlaneFormatter(state.plane);
        Position center =
                GcodePreprocessorUtils.updateCenterWithCommand(
                        args, state.currentPoint, nextPoint, state.inAbsoluteIJKMode, clockwise, plane);

        double radius = GcodePreprocessorUtils.parseCoord(args, 'R');

        // Calculate radius if necessary, according to the current G17/18/19 Plane
        if (Double.isNaN(radius)) {

            radius = Math.sqrt(
                    Math.pow(plane.axis0(state.currentPoint) - plane.axis0(center), 2.0)
                            + Math.pow(plane.axis1(state.currentPoint) - plane.axis1(center), 2.0));
        }

        ps.setIsMetric(state.isMetric);
        ps.setArcCenter(center);
        ps.setIsArc(true);
        ps.setRadius(radius);
        ps.setIsClockwise(clockwise);
        ps.setPlaneState(state.plane);

        boolean isRotation = state.currentPoint.hasRotationTo(nextPoint);
        ps.setIsRotation(isRotation);


        // Save off the endpoint.
        state.currentPoint = nextPoint;
        return ps;
    }

    /**
     * Branch parser to handle specific gcode command.
     * <p>
     * A copy of the state object should go in the resulting GcodeMeta object.
     */
    private static GcodeParser.GcodeMeta handleGCode(final Code code, List<String> args, int line, GcodeState state,
                                                     boolean machineCoordinates)
            throws GcodeParserException {
        GcodeParser.GcodeMeta meta = new GcodeParser.GcodeMeta();

        meta.code = code;

        Position previousPoint = state.currentPoint;
        Position nextPoint = null;

        // If it is a movement code make sure it has some coordinates.
        if (code.consumesMotion()) {
            nextPoint = GcodePreprocessorUtils.updatePointWithCommand(args, state.currentPoint, state.inAbsoluteMode);

            if (nextPoint == null) {
                if (!code.motionOptional()) {
                    throw new GcodeParserException(
                            Localization.getString("parser.gcode.missing-axis-commands") + ": " + code);
                }
            }
        }

        if (nextPoint == null && meta.point != null) {
            nextPoint = meta.point.point();
        }

        switch (code) {
            case G0:
                meta.point = addLinearPointSegment(nextPoint, true, line, state);
                updateKnownAxes(args, state, machineCoordinates, previousPoint);
                break;
            case G1:
                meta.point = addLinearPointSegment(nextPoint, false, line, state);
                updateKnownAxes(args, state, machineCoordinates, previousPoint);
                break;

            // Arc command.
            case G2:
                meta.point = addArcPointSegment(nextPoint, true, args, line, state);
                updateKnownAxes(args, state, machineCoordinates, previousPoint);
                break;
            case G3:
                meta.point = addArcPointSegment(nextPoint, false, args, line, state);
                updateKnownAxes(args, state, machineCoordinates, previousPoint);
                break;

            // Canned cycles (also their modal repeats, like a line with only "X3 Y3") and spindle synchronized
            // motion are not modelled, so the position after them is unknown.
            case G33:
            case G73:
            case G76:
            case G81:
            case G82:
            case G83:
            case G84:
            case G85:
            case G86:
            case G87:
            case G88:
            case G89:
                state.unknownAxes.addAll(XYZ);
                break;

            // Go to a stored position. Only the named axes move, or all axes if none are named.
            case G28:
            case G28_2:
            case G28_3:
            case G30: {
                EnumSet<Axis> axes = axisWords(args);
                state.unknownAxes.addAll(axes.isEmpty() ? EnumSet.allOf(Axis.class) : axes);
                break;
            }

            // The work coordinate frame of the named axes changes.
            case G10:
                if (changesActiveWorkOffset(args, state)) {
                    state.unknownAxes.addAll(axisWords(args));
                    state.workFrameChanged = true;
                }
                break;
            case G92:
                state.unknownAxes.addAll(axisWords(args));
                state.workFrameChanged = true;
                break;
            case G92_1:
            case G92_2:
            case G92_3:
                state.unknownAxes.addAll(EnumSet.allOf(Axis.class));
                state.workFrameChanged = true;
                break;

            // Tool length offset changes the Z frame.
            case G43:
            case G43_1:
            case G49:
                state.unknownAxes.add(Axis.Z);
                break;

            case G17:
            case G18:
            case G19:
            case G17_1:
            case G18_1:
            case G19_1:
                state.plane = Plane.lookup(code);
                break;

            //inch
            case G20:
                state.isMetric = false;
                state.units = G20;
                state.currentPoint = state.currentPoint.getPositionIn(UnitUtils.Units.INCH);
                break;
            //mm
            case G21:
                state.isMetric = true;
                state.units = G21;
                state.currentPoint = state.currentPoint.getPositionIn(UnitUtils.Units.MM);
                break;

            // Probe: http://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g38
            case G38_2: // probe toward workpiece, stop on contact, signal error if failure
            case G38_3: // probe toward workpiece, stop on contact
            case G38_4: // probe away from workpiece, stop on loss of contact, signal error if failure
            case G38_5: // probe away from workpiece, stop on loss of contact
                meta.point = addProbePointSegment(nextPoint, true, line, state);
                // The probe stops at an unknown contact point, not at the target.
                state.unknownAxes.addAll(axisWords(args));
                break;

            // These are not used in the visualizer.
            case G54:
            case G55:
            case G56:
            case G57:
            case G58:
            case G59:
            case G59_1:
            case G59_2:
            case G59_3:
                if (state.offset != code) {
                    state.unknownAxes.addAll(EnumSet.allOf(Axis.class));
                    // Selecting the work coordinate system before the first move is not a change of frame
                    if (state.motionStarted) {
                        state.workFrameChanged = true;
                    }
                }
                state.offset = code;
                break;

            case G90:
                state.inAbsoluteMode = true;
                state.distanceMode = G90;
                break;
            case G91:
                state.inAbsoluteMode = false;
                state.distanceMode = G91;
                break;

            case G90_1:
                state.inAbsoluteIJKMode = true;
                state.arcDistanceMode = G90_1;
                break;
            case G91_1:
                state.inAbsoluteIJKMode = false;
                state.arcDistanceMode = G91_1;
                break;

            case G93:
            case G94:
            case G95:
                state.feedMode = code;
                break;
            default:
                break;
        }
        if (code.getType() == Motion) {
            state.currentMotionMode = code;
        }
        meta.state = state.copy();
        return meta;
    }

    /**
     * Axis words on a G0/G1/G2/G3 line in absolute mode make those axes known again. With G53 they are machine
     * coordinates, so the axes become unknown. A relative move leaves them as they were, except that a relative
     * move from a position that was never set stays unknown (the parser would otherwise start it from zero).
     */
    private static void updateKnownAxes(List<String> args, GcodeState state, boolean machineCoordinates,
                                        Position previousPoint) {
        EnumSet<Axis> axes = axisWords(args);
        // A move in machine coordinates (a G53 retract in the program header) is not motion in a work frame
        if (!machineCoordinates && !axes.isEmpty()) {
            state.motionStarted = true;
        }
        if (machineCoordinates) {
            state.unknownAxes.addAll(axes);
        } else if (state.inAbsoluteMode) {
            state.unknownAxes.removeAll(axes);
        } else {
            for (Axis axis : axes) {
                if (previousPoint == null || Double.isNaN(previousPoint.get(axis))) {
                    state.unknownAxes.add(axis);
                }
            }
        }
    }

    /**
     * G10 L2/L20 sets the offsets of the work coordinate system given by P (P0 is the active one, P1 is G54 ...
     * P9 is G59.3). Other G10 forms (L1, L10, L11) change the tool table, which doesn't move the active frame.
     */
    private static boolean changesActiveWorkOffset(List<String> args, GcodeState state) {
        double l = GcodePreprocessorUtils.parseCoord(args, 'L');
        if (l != 2 && l != 20) {
            return false;
        }
        double p = GcodePreprocessorUtils.parseCoord(args, 'P');
        if (Double.isNaN(p) || p == 0) {
            return true;
        }
        Code[] systems = {Code.G54, Code.G55, Code.G56, Code.G57, Code.G58, Code.G59, Code.G59_1, Code.G59_2, Code.G59_3};
        int index = (int) p - 1;
        return index < 0 || index >= systems.length || systems[index] == state.offset;
    }

    private static boolean hasMCode(List<String> args, int code) {
        for (String value : GcodePreprocessorUtils.parseCodes(args, 'M')) {
            try {
                if (Double.parseDouble(value) == code) {
                    return true;
                }
            } catch (NumberFormatException e) {
                // Not a number, not this code
            }
        }
        return false;
    }

    private static EnumSet<Axis> axisWords(List<String> args) {
        EnumSet<Axis> axes = EnumSet.noneOf(Axis.class);
        for (Axis axis : Axis.values()) {
            if (!Double.isNaN(GcodePreprocessorUtils.parseCoord(args, axis.name().charAt(0)))) {
                axes.add(axis);
            }
        }
        return axes;
    }

    /**
     * Helper method to apply processors to gcode.
     */
    public static void processAndExport(GcodeParser gcp, File input, IGcodeWriter output)
            throws IOException, GcodeParserException {
        try (InputStream inputStream = new FileInputStream(input)) {
            if (processAndExportGcodeStream(gcp, inputStream, output)) {
                return;
            }
        }

        try (BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(input), StandardCharsets.UTF_8.name()))) {
            processAndExportText(gcp, br, output);
        }
    }

    /**
     * Common logic in processAndExport* methods.
     */
    private static void preprocessAndWrite(GcodeParser gcp, IGcodeWriter gsw, String command, String comment, int idx) throws GcodeParserException {
        if (idx % 100000 == 0) {
            LOGGER.log(Level.FINE, "gcode processing line: " + idx);
        }

        // Parse the gcode for the buffer.
        Collection<String> lines = gcp.preprocessCommand(command, gcp.getCurrentState());

        for (String processedLine : lines) {
            gsw.addLine(command, processedLine, comment, idx);
        }

        gcp.addCommand(command);
    }

    /**
     * Attempts to read the input file in GcodeStream format.
     *
     * @return whether or not we succeed processing the file.
     */
    private static boolean processAndExportGcodeStream(GcodeParser gcp, InputStream input, IGcodeWriter output)
            throws IOException, GcodeParserException {

        // Preprocess a GcodeStream file.
        try (IGcodeStreamReader gsr = new GcodeStreamReader(input, new DefaultCommandCreator())) {
            int i = 0;
            while (gsr.getNumRowsRemaining() > 0) {
                i++;
                GcodeCommand gc = gsr.getNextCommand();
                preprocessAndWrite(gcp, output, gc.getCommandString(), gc.getComment(), i);
            }

            // Done processing GcodeStream file.
            return true;
        } catch (GcodeStreamReader.NotGcodeStreamFile ex) {
            // File exists, but isn't a stream reader. So go ahead and try parsing it as a raw gcode file.
        }
        return false;
    }

    /**
     * Attempts to read the input file in gcode-text format.
     *
     * @return whether or not we succeed processing the file.
     */
    private static void processAndExportText(GcodeParser gcp, BufferedReader input, IGcodeWriter output)
            throws IOException, GcodeParserException {
        // Preprocess a regular gcode file.
        try (BufferedReader br = input) {
            int i = 0;
            for (String line; (line = br.readLine()) != null; ) {
                i++;

                String comment = GcodePreprocessorUtils.parseComment(line);
                preprocessAndWrite(gcp, output, line, comment, i);
            }
        }
    }
}
