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

import com.willwinder.universalgcodesender.gcode.GcodeParser.GcodeMeta;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import com.willwinder.universalgcodesender.gcode.util.Code;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserException;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserUtils;
import com.willwinder.universalgcodesender.model.PartialPosition;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Adjust Z heights based on a provided surface mesh.
 *
 * @author wwinder
 */
public class MeshLeveler implements CommandProcessor {
    public final static String ERROR_MESH_SHAPE = "Surface mesh must be a rectangular 2D array.";
    public final static String ERROR_NOT_ENOUGH_SAMPLES = "Need at least 2 samples along each axis.";
    public final static String ERROR_X_ALIGNMENT = "Unaligned x coordinate in surface grid.";
    public final static String ERROR_Y_ALIGNMENT = "Unaligned y coordinate in surface grid.";
    public final static String ERROR_Y_ASCENTION = "Found a y coordinate that isn't ascending.";
    public final static String ERROR_X_ASCENTION = "Found a x coordinate that isn't ascending.";
    public final static String ERROR_UNEXPECTED_ARC = "The mesh leveler cannot process arcs. Enable the arc expander.";
    public final static String ERROR_MISSING_POINT_DATA = "Internal parser error: missing data. ";
    final private double materialSurfaceHeightMM;
    final private Position[][] surfaceMesh;
    final private Position lowerLeft;
    final private Position upperRight;
    final private int xLen, yLen;
    final private double xResolution, yResolution;
    private Units surfaceMeshUnits;

    /**
     * @param materialSurfaceHeightMM Z height used in offset.
     * @param surfaceMesh             2D array in the format Position[x][y]
     */
    public MeshLeveler(double materialSurfaceHeightMM, Position[][] surfaceMesh) {
        if (surfaceMesh == null) {
            throw new IllegalArgumentException("Surface mesh is required.");
        }

        this.materialSurfaceHeightMM = materialSurfaceHeightMM;
        this.yLen = surfaceMesh[0].length;
        this.xLen = surfaceMesh.length;

        validateMesh(surfaceMesh);
        this.surfaceMesh = surfaceMesh;
        this.surfaceMeshUnits = surfaceMesh[0][0].getUnits();
        this.xResolution = surfaceMesh[1][0].x - surfaceMesh[0][0].x;
        this.yResolution = surfaceMesh[0][1].y - surfaceMesh[0][0].y;

        this.lowerLeft = surfaceMesh[0][0];
        this.upperRight = surfaceMesh[xLen - 1][yLen - 1];
    }

    private void validateMesh(Position[][] surfaceMesh) {
        // Validate that points form a rectangular 2D array.
        for (Position[] arr : surfaceMesh) {
            if (arr.length != yLen) {
                throw new IllegalArgumentException(ERROR_MESH_SHAPE);
            }
        }

        // Need at least 4 points for bilinear interpolation.
        if (xLen < 2 || yLen < 2) {
            throw new IllegalArgumentException(ERROR_NOT_ENOUGH_SAMPLES);
        }

        // Verify that x points are aligned and y points are ascending.
        for (int xIdx = 0; xIdx < xLen; xIdx++) {
            double xCoord = surfaceMesh[xIdx][0].x;
            double yCoord = surfaceMesh[xIdx][0].y;
            for (int yIdx = 0; yIdx < yLen; yIdx++) {
                if (surfaceMesh[xIdx][yIdx].x != xCoord) {
                    String err = "@ " + xIdx + ", " + yIdx + ": (" + xCoord + " != " + surfaceMesh[xIdx][yIdx].x + ")";
                    throw new IllegalArgumentException(ERROR_X_ALIGNMENT + err);
                }
                if (yCoord > surfaceMesh[xIdx][yIdx].y) {
                    String err = "@ " + xIdx + ", " + yIdx + ": (" + yCoord + " !<= " + surfaceMesh[xIdx][yIdx].y + ")";
                    throw new IllegalArgumentException(ERROR_Y_ASCENTION + err);
                }
                yCoord = surfaceMesh[xIdx][yIdx].y;
            }
        }

        // Verify that y points are aligned and x points are ascending.
        for (int yIdx = 0; yIdx < yLen; yIdx++) {
            double xCoord = surfaceMesh[0][yIdx].x;
            double yCoord = surfaceMesh[0][yIdx].y;
            for (int xIdx = 0; xIdx < xLen; xIdx++) {
                if (surfaceMesh[xIdx][yIdx].y != yCoord) {
                    String err = "@ " + xIdx + ", " + yIdx + ": (" + yCoord + " != " + surfaceMesh[xIdx][yIdx].y + ")";
                    throw new IllegalArgumentException(ERROR_Y_ALIGNMENT + err);
                }
                if (xCoord > surfaceMesh[xIdx][yIdx].x) {
                    String err = "@ " + xIdx + ", " + yIdx + ": (" + xCoord + " !<= " + surfaceMesh[xIdx][yIdx].x + ")";
                    throw new IllegalArgumentException(ERROR_X_ASCENTION + err);
                }
                xCoord = surfaceMesh[xIdx][yIdx].x;
            }
        }
    }

    private boolean ensureJustLines(List<GcodeMeta> commands) throws GcodeParserException {
        if (commands == null) return false;
        boolean hasLine = false;
        for (GcodeMeta command : commands) {
            switch (command.code) {
                case G0:
                case G1:
                    hasLine = true;
                    break;
                case G2:
                case G3:
                    throw new GcodeParserException(ERROR_UNEXPECTED_ARC);
            }
        }
        return hasLine;
    }

    @Override
    public List<String> processCommand(final String commandString, GcodeState state) throws GcodeParserException {
        List<GcodeMeta> commands = GcodeParserUtils.processCommand(commandString, 0, state);

        if (commands == null) {
            return Collections.singletonList(commandString);
        }

        // A move in machine coordinates (G53), a line that goes home, probes or changes the work coordinate
        // frame or tool offset, or that moves a rotary axis, is passed through untouched: a work-coordinate surface
        // offset does not apply to it. The ArcExpander leaves such arcs, and arcs that start at an unknown
        // position, unexpanded; they cannot be leveled either.
        boolean isArc = commands.stream().anyMatch(c -> c.code == Code.G2 || c.code == Code.G3);
        if (CoordinateRewriteGuard.mustPassThrough(commandString, commands, state)
                || (isArc && !state.isPositionKnownXYZ())) {
            return Collections.singletonList(commandString);
        }

        // The mesh was scanned in the work coordinate frame the program started in. After G92, G10 or a switch to
        // another work coordinate system the rest of the program is not leveled.
        if (state.workFrameChanged) {
            return Collections.singletonList(commandString);
        }

        // If there are no lines, return unmodified input.
        if (!ensureJustLines(commands)) {
            return Collections.singletonList(commandString);
        }

        // Other G-codes on the line (G90, G17, ...) have their own meta without a point. Only the G0/G1 meta is
        // adjusted and the line is emitted once.
        GcodeMeta command = commands.stream()
                .filter(c -> c.code == Code.G0 || c.code == Code.G1)
                .findFirst()
                .orElse(null);
        if (command == null) {
            throw new GcodeParserException(ERROR_MISSING_POINT_DATA + commandString);
        }
        if (command.point == null) {
            // G0/G1 without axis words, for example "G1 F100"
            return Collections.singletonList(commandString);
        }

        // Use the distance mode that applies to this move, including a G90/G91 on the same line.
        boolean absolute = command.state.inAbsoluteMode;

        // Leave the move untouched if its end (or for a relative move, its start) is not a known work position:
        // the first moves of a program, and moves after G53, G28/G30, G38.x, G92, G10, G43.1 or a change of work
        // coordinate system. Leveling it would add a Z word calculated from a guessed position.
        if (command.state.workFrameChanged || !command.state.isPositionKnownXYZ()
                || (!absolute && !state.isPositionKnownXYZ())) {
            return Collections.singletonList(commandString);
        }

        Position start = state.currentPoint;
        Position end = command.point.point();

        // Get offset relative to the expected surface height.
        // Visualizer normalizes everything to MM but probe mesh might be INCH
        double zPointOffset;
        if (absolute) {
            Position position = end.getPositionIn(surfaceMeshUnits);
            double materialSurfaceHeight = this.materialSurfaceHeightMM * UnitUtils.scaleUnits(Units.MM, surfaceMeshUnits);
            zPointOffset = (surfaceHeightAt(position.x, position.y) - materialSurfaceHeight) * UnitUtils.scaleUnits(surfaceMeshUnits, end.getUnits());
        } else {
            // TODO: If the first move in the gcode file is relative it won't properly take the materialSurfaceHeight
            // into account. To fix the CommandProcessor needs to inject an adjustment before that first relative move
            // happens. Until that happens the user must make sure the materialSurfaceHeight is zero.

            // In relative mode we only need to adjust by the z delta between the starting and ending point
            Position startPositionInMeshUnits = start.getPositionIn(surfaceMeshUnits);
            double startHeight = surfaceHeightAt(startPositionInMeshUnits.x, startPositionInMeshUnits.y);
            Position endPositionInMeshUnits = end.getPositionIn(surfaceMeshUnits);
            double endHeight = surfaceHeightAt(endPositionInMeshUnits.x, endPositionInMeshUnits.y);
            zPointOffset = (endHeight - startHeight) * UnitUtils.scaleUnits(surfaceMeshUnits, end.getUnits());
        }

        // Update z coordinate.
        double newZ = end.getZ() + zPointOffset;

        PartialPosition.Builder overrideZ = PartialPosition.builder(end.getUnits());
        if (absolute) {
            overrideZ.setZ(newZ);
        } else {
            overrideZ.setZ(newZ - start.getPositionIn(end.getUnits()).getZ());
        }

        return Collections.singletonList(GcodePreprocessorUtils.overridePosition(withoutComments(commandString), overrideZ.build()));
    }

    /**
     * Removes comments so a Z inside a comment is not rewritten and the new Z is not appended inside a ';' comment.
     */
    private static String withoutComments(String command) {
        if (command.indexOf('(') < 0 && command.indexOf(';') < 0) {
            return command;
        }
        return GcodePreprocessorUtils.splitCommand(command).stream()
                .filter(word -> !word.startsWith("(") && !word.startsWith(";"))
                .collect(Collectors.joining());
    }

    protected Position[][] findBoundingArea(double x, double y) {
        double xOffset = x - this.lowerLeft.x;
        double yOffset = y - this.lowerLeft.y;

        int xIdx = (int) ((xOffset == 0) ? 0 : (xOffset / this.xResolution));
        int yIdx = (int) ((yOffset == 0) ? 0 : (yOffset / this.yResolution));

        // Clamp bounds
        xIdx = Math.min(xIdx, this.xLen - 2);
        yIdx = Math.min(yIdx, this.yLen - 2);
        xIdx = Math.max(xIdx, 0);
        yIdx = Math.max(yIdx, 0);

        return new Position[][]{
                {this.surfaceMesh[xIdx][yIdx], this.surfaceMesh[xIdx][yIdx + 1]},
                {this.surfaceMesh[xIdx + 1][yIdx], this.surfaceMesh[xIdx + 1][yIdx + 1]}
        };
    }

    /**
     * Get the surface height from the mesh and returns it in the surface mesh units.
     * <p>
     * Bilinear interpolation:
     * http://supercomputingblog.com/graphics/coding-bilinear-interpolation/
     */
    protected double surfaceHeightAt(double x, double y) {
        // Outside the probed area use the height at the nearest edge instead of extrapolating the edge slope.
        x = Math.max(this.lowerLeft.x, Math.min(x, this.upperRight.x));
        y = Math.max(this.lowerLeft.y, Math.min(y, this.upperRight.y));

        Position[][] q = findBoundingArea(x, y);

        Position Q11 = q[0][0];
        Position Q21 = q[1][0];
        Position Q12 = q[0][1];
        Position Q22 = q[1][1];

        double x1 = Q11.x;
        double x2 = Q21.x;
        double y1 = Q11.y;
        double y2 = Q12.y;

        double R1 = ((x2 - x) / (x2 - x1)) * Q11.z + ((x - x1) / (x2 - x1)) * Q21.z;
        double R2 = ((x2 - x) / (x2 - x1)) * Q12.z + ((x - x1) / (x2 - x1)) * Q22.z;

        return ((y2 - y) / (y2 - y1)) * R1 + ((y - y1) / (y2 - y1)) * R2;
    }

    @Override
    public String getHelp() {
        return null;
    }
}
