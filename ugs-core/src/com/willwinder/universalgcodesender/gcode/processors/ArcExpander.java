/*
    Copyright 2016-2020 Will Winder

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
package com.willwinder.universalgcodesender.gcode.processors;

import com.willwinder.universalgcodesender.gcode.GcodeParser.GcodeMeta;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils.SplitCommand;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import com.willwinder.universalgcodesender.gcode.util.Code;
import static com.willwinder.universalgcodesender.gcode.util.Code.G1;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserException;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserUtils;
import com.willwinder.universalgcodesender.gcode.util.Plane;
import com.willwinder.universalgcodesender.gcode.util.PlaneFormatter;
import com.willwinder.universalgcodesender.i18n.Localization;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.types.PointSegment;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Expand an arc into smaller sections. You can configure the length of each
 * section, and whether it is expanded with a bunch of smaller arcs, or with
 * line segments.
 *
 * @author wwinder
 */
public class ArcExpander implements CommandProcessor {
    private final boolean convertToLines;
    private final double length;
    private final DecimalFormat df;
    private final boolean requireKnownPosition;

    @Override
    public String getHelp() {
        return Localization.getString("sender.help.arcs") + "\n"
                + Localization.getString("sender.arcs.length")
                + ": " + df.format(length);
    }

    /**
     * @param convertToLines toggles if smaller lines or arcs are returned.
     * @param length the length of each smaller segment.
     */
    public ArcExpander(boolean convertToLines, double length) {
        this.convertToLines = convertToLines;
        this.length = length;

        // Setup decimal formatter
        df = new DecimalFormat("#.#########", Localization.dfs);
        requireKnownPosition = false;
    }

    /**
     * @param convertToLines toggles if smaller lines or arcs are returned.
     * @param length the length of each smaller segment.
     */
    public ArcExpander(boolean convertToLines, double length, DecimalFormat df) {
        this(convertToLines, length, df, false);
    }

    /**
     * @param convertToLines       toggles if smaller lines or arcs are returned.
     * @param length               the length of each smaller segment.
     * @param requireKnownPosition only expand arcs whose start is a known position in the work coordinate system,
     *                             and leave lines in machine coordinates, after a tool change etc. as they are. Used
     *                             when the segments are rewritten again afterwards, like by the mesh leveler.
     */
    public ArcExpander(boolean convertToLines, double length, DecimalFormat df, boolean requireKnownPosition) {
        this.convertToLines = convertToLines;
        this.length = length;
        this.df = df;
        this.requireKnownPosition = requireKnownPosition;
    }

    @Override
    public List<String> processCommand(String command, GcodeState state) throws GcodeParserException {
        if (state.currentPoint == null) throw new GcodeParserException(Localization.getString("parser.processor.arc.start-error"));

        List<String> results = new ArrayList<>();

        List<GcodeMeta> commands = GcodeParserUtils.processCommand(command, 0, state, true);

        // If this is not an arc, there is nothing to do.
        Code c = hasArcCommand(commands);
        if (c == null) {
            return Collections.singletonList(command);
        }

        GcodeMeta arcMeta = commands.stream().filter(m -> m.code == c).findFirst().orElseThrow(IllegalStateException::new);
        PointSegment ps = arcMeta.point;
        Position start = new Position(state.currentPoint);
        Position end = new Position(arcMeta.point.point());
        PlaneFormatter plane = new PlaneFormatter(ps.getPlaneState());
        Axis[] planeAxes = planeAxes(ps.getPlaneState());

        if (requireKnownPosition) {
            if (CoordinateRewriteGuard.mustPassThrough(command, commands, state)) {
                return passThrough(command, c);
            }

            // Absolute segments, or an absolute arc center, need the start of the arc. If it is unknown in the plane
            // (after G53/G28/G30/G38.x/G92/G10, M6, M0, a change of work coordinate system) leave the arc as it is
            // instead of generating segments from a guessed start.
            boolean needsStart = arcMeta.state.inAbsoluteMode || arcMeta.state.inAbsoluteIJKMode;
            if (needsStart && (!state.isPositionKnown(planeAxes[0]) || !state.isPositionKnown(planeAxes[1]))) {
                return passThrough(command, c);
            }

            // If the axis across the plane is unknown (a 2D program, or Z after G28), expand the arc in its plane
            // without writing that axis. A helical arc that moves it can't be expanded.
            if (needsStart && !state.isPositionKnown(planeAxes[2])) {
                if (!Double.isNaN(GcodePreprocessorUtils.parseCoord(GcodePreprocessorUtils.splitCommand(command), planeAxes[2].name().charAt(0)))) {
                    return passThrough(command, c);
                }
                plane.setLinear(start, Double.NaN);
                plane.setLinear(end, Double.NaN);
            }
        }

        // Without a start in the plane there is nothing to expand from. Keep the arc instead of dropping it.
        if (Double.isNaN(plane.axis0(start)) || Double.isNaN(plane.axis1(start))) {
            return passThrough(command, c);
        }

        List<Position> points = GcodePreprocessorUtils.generatePointsAlongArcBDring(
                start, end, ps.center(), ps.isClockwise(),
                ps.getRadius(), 0, this.length, plane);

        SplitCommand sc = GcodePreprocessorUtils.extractMotion(c, command);
        if (sc == null) {
            return Collections.singletonList(command);
        }

        // Settings like G90 or F run before the segments, a program stop (M0, M30, ...) after them.
        String[] remainder = CoordinateRewriteGuard.splitProgramStops(sc.remainder);
        if (!remainder[0].isEmpty()) {
            results.add(remainder[0]);
        }

        if (points.isEmpty()) {
            if (!remainder[1].isEmpty()) {
                results.add(remainder[1]);
            }
            return results;
        }

        // That function returns the first and last points. Exclude the first
        // point because the previous gcode command ends there already.
        points.remove(0);

        if (convertToLines) {
            for (Position point : points) {
                results.add(GcodePreprocessorUtils.generateLineFromPoints(G1, start, point, arcMeta.state.inAbsoluteMode, df));
                start = point;
            }
        } else {
            // TODO: Generate arc segments.
            throw new UnsupportedOperationException("I have not implemented this.");
        }

        if (!remainder[1].isEmpty()) {
            results.add(remainder[1]);
        }
        return results;
    }

    /**
     * Returns the arc line as it is. A line that continues the arc mode without its own G2/G3 word gets one: after an
     * earlier arc was expanded into G1 lines the controller is no longer in arc mode.
     */
    private static List<String> passThrough(String command, Code arc) {
        if (GcodePreprocessorUtils.getGCodes(GcodePreprocessorUtils.splitCommand(command)).contains(arc)) {
            return Collections.singletonList(command);
        }
        return Collections.singletonList(arc + command);
    }

    /**
     * @return the two axes of the plane and the axis across it
     */
    private static Axis[] planeAxes(Plane plane) {
        switch (plane) {
            case ZX:
                return new Axis[]{Axis.Z, Axis.X, Axis.Y};
            case YZ:
                return new Axis[]{Axis.Y, Axis.Z, Axis.X};
            case XY:
            default:
                return new Axis[]{Axis.X, Axis.Y, Axis.Z};
        }
    }

    private static Code hasArcCommand(List<GcodeMeta> commands) {
        if (commands == null) return null;
        for (GcodeMeta meta : commands) {
            if (meta.point != null && meta.point.isArc()) {
                return meta.code;
            }
        }
        return null;
    }
}
