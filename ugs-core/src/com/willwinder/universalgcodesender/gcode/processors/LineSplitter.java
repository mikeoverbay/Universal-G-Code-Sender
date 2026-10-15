/*
    Copyright 2017-2020 Will Winder

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

import com.willwinder.universalgcodesender.gcode.GcodeParser;
import com.willwinder.universalgcodesender.gcode.GcodeParser.GcodeMeta;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils.SplitCommand;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import com.willwinder.universalgcodesender.gcode.util.Code;
import static com.willwinder.universalgcodesender.gcode.util.Code.*;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserException;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserUtils;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Split lines into a series of smaller line segments.
 *
 * @author wwinder
 */
public class LineSplitter implements CommandProcessor {
    final double maxSegmentLength;
    private final Units segmentLengthUnits;

    /**
     * @param segmentLength length of longest single line, in the units of the program.
     */
    public LineSplitter(double segmentLength) {
        this(segmentLength, null);
    }

    /**
     * @param segmentLength length of longest single line.
     * @param units         the units of segmentLength, or null for the units of the program.
     */
    public LineSplitter(double segmentLength, Units units) {
        this.maxSegmentLength = segmentLength;
        this.segmentLengthUnits = units;
    }

    @Override
    public String getHelp() {
        return "Split G0 and G1 commands into multiple commands.";
    }

    private Code hasLine(List<GcodeMeta> commands) {
        if (commands == null) return null;
        for (GcodeMeta command : commands) {
            switch(command.code){
                case G0:
                    return G0;
                case G1:
                    return G1;
            }
        }
        return null;
    }

    @Override
    public List<String> processCommand(String commandString, GcodeState state) throws GcodeParserException {
        List<GcodeParser.GcodeMeta> commands = GcodeParserUtils.processCommand(commandString, 0, state);

        Code code = hasLine(commands);
        if (code == null) {
            return Collections.singletonList(commandString);
        }

        // Never rewrite a move in machine coordinates (G53) or a line that goes home, probes or changes the work
        // coordinate frame or tool offset: the generated segments would be plain work-coordinate moves.
        if (CoordinateRewriteGuard.mustPassThrough(commandString, commands, state)) {
            return Collections.singletonList(commandString);
        }

        GcodeMeta command = commands.stream()
                .filter(c -> c.code == code)
                .findFirst()
                .orElseThrow(() -> new GcodeParserException("Internal parser error: missing data. " + commandString));
        if (command.point == null) {
            // No point data associated with this command (Maybe just setting feed rate), leave it as-is
            return Collections.singletonList(commandString);
        }

        // Absolute segments are only written when both ends are known positions in the work coordinate system.
        // Otherwise every segment would get X, Y and Z words calculated from a guessed position (first move of the
        // program, or after G53/G28/G30/G38.x/G92/G10/G43.1/M6/M0 or a change of work coordinate system).
        // Relative segments only depend on the distance moved, so they can be written from any start.
        if (command.state.inAbsoluteMode && (!state.isPositionKnownXYZ() || !command.state.isPositionKnownXYZ())) {
            return Collections.singletonList(commandString);
        }

        // line length
        Position end = command.point.point();
        Position start = state.currentPoint.getPositionIn(end.getUnits());
        Position current = start;
        double length = start.distanceXYZ(end);
        double maxLength = segmentLengthUnits == null
                ? this.maxSegmentLength
                : this.maxSegmentLength * UnitUtils.scaleUnits(segmentLengthUnits, end.getUnits());

        // Check if line needs splitting. The length is not a number when an axis was never set.
        if (!(maxLength > 0) || !(length > maxLength)) {
            return Collections.singletonList(commandString);
        }

        SplitCommand sc = GcodePreprocessorUtils.extractMotion(code, commandString);
        if (sc == null) {
            return Collections.singletonList(commandString);
        }

        // Settings like G90 or F run before the segments, a program stop (M0, M30, ...) after them.
        String[] remainder = CoordinateRewriteGuard.splitProgramStops(sc.remainder);
        List<String> results = new ArrayList<>();
        if (!remainder[0].isEmpty()) {
            results.add(remainder[0]);
        }

        int numSegments = (int) Math.ceil(length / maxLength);
        double segmentLength = length / Math.ceil(length / maxLength);

        // Create line segments, stop before the last one which uses the end point.
        for (int i = 1; i < numSegments; i++) {
            double k = 1 / (length / (i * segmentLength));
            double newX = start.x + k * (end.x - start.x);
            double newY = start.y + k * (end.y - start.y);
            double newZ = start.z + k * (end.z - start.z);

            Position next = new Position(newX, newY, newZ, start.getUnits());
            results.add(GcodePreprocessorUtils.generateLineFromPoints(
                    command.code, current, next, command.state.inAbsoluteMode, null));
            current = next;
        }

        // Add the last line point.
        results.add(GcodePreprocessorUtils.generateLineFromPoints(
                command.code, current, end, command.state.inAbsoluteMode, null));

        if (!remainder[1].isEmpty()) {
            results.add(remainder[1]);
        }
        return results;
    }
}
