/*
    Copyright 2023-2024 Will Winder

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
package com.willwinder.ugs.platform.surfacescanner;

import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.processors.ArcExpander;
import com.willwinder.universalgcodesender.gcode.processors.CommandProcessorList;
import com.willwinder.universalgcodesender.gcode.processors.LineSplitter;
import com.willwinder.universalgcodesender.gcode.processors.MeshLeveler;
import com.willwinder.universalgcodesender.i18n.Localization;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.utils.AutoLevelSettings;

import javax.swing.*;
import java.awt.*;
import java.util.List;

/**
 * Static utils
 */
public class Utils {

    public static final JFileChooser fileChooser = new JFileChooser();

    public static boolean shouldEraseProbedData() {
        int result = JOptionPane.showConfirmDialog(new Frame(),
                Localization.getString("autoleveler.panel.overwrite"),
                Localization.getString("AutoLevelerTitle"),
                JOptionPane.YES_NO_OPTION);
        return result == JOptionPane.YES_OPTION;
    }

    public static Position getMaxPosition(List<Position> positions) {
        double maxX = -Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        double maxZ = -Double.MAX_VALUE;
        UnitUtils.Units units = UnitUtils.Units.MM;
        for (Position p : positions) {
            units = p.getUnits();
            maxX = Math.max(maxX, p.getX());
            maxY = Math.max(maxY, p.getY());
            maxZ = Math.max(maxZ, p.getZ());
        }

        return new Position(maxX, maxY, maxZ, units);
    }

    public static Position getMinPosition(List<Position> positions) {
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double minZ = Double.MAX_VALUE;
        UnitUtils.Units units = UnitUtils.Units.MM;
        for (Position p : positions) {
            units = p.getUnits();
            minX = Math.min(minX, p.getX());
            minY = Math.min(minY, p.getY());
            minZ = Math.min(minZ, p.getZ());
        }

        return new Position(minX, minY, minZ, units);
    }

    public static Position getRoundPosition(Position position) {
        double x = Math.round(position.getX() * 1000d) / 1000d;
        double y = Math.round(position.getY() * 1000d) / 1000d;
        double z = Math.round(position.getZ() * 1000d) / 1000d;
        return new Position(x, y, z, position.getUnits());
    }

    /**
     * @param preferredUnits the units the auto level settings (like the Z surface) are shown in
     */
    public static CommandProcessorList createCommandProcessor(AutoLevelSettings autoLevelSettings, SurfaceScanner surfaceScanner, UnitUtils.Units preferredUnits) {
        Position[][] grid = surfaceScanner.getProbePositionGrid();
        if (grid.length < 2 || grid[0].length < 2) {
            throw new IllegalArgumentException(MeshLeveler.ERROR_NOT_ENOUGH_SAMPLES);
        }

        CommandProcessorList result = new CommandProcessorList();

        // Step 1: Convert arcs to line segments, only from known positions since they are leveled afterwards.
        result.add(new ArcExpander(true, autoLevelSettings.getAutoLevelArcSliceLength(), GcodePreprocessorUtils.getDecimalFormatter(), true));

        // Step 2: Line splitter. No line should be longer than a fraction of the probe spacing, measured on the
        // probe grid in its own units (the program may be in other units).
		double stepResX = grid[1][0].x - grid[0][0].x;
		double stepResY = grid[0][1].y - grid[0][0].y;
		double avgRes = (stepResX + stepResY) / 2.0;
		if (avgRes > 0) {
			result.add(new LineSplitter(avgRes / 4, grid[0][0].getUnits()));
		}

        // Step 3: Adjust Z heights codes based on mesh offsets. The Z surface is entered in the preferred units,
        // the MeshLeveler takes it in mm.
        double zSurfaceMM = autoLevelSettings.getZSurface() * UnitUtils.scaleUnits(preferredUnits, UnitUtils.Units.MM);
        result.add(new MeshLeveler(zSurfaceMM, grid));
        return result;
    }
	public static String extractMetadataValue(String line, String key) {
    if (line.startsWith("#")) {
        String[] parts = line.substring(1).trim().split("\\s+");
        for (String part : parts) {
            if (part.startsWith(key + "=")) {
                return part.substring((key + "=").length());
            }
        }
    }
    return null;
}

}
