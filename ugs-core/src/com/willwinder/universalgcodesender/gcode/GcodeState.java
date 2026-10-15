/*
    Copyright 2016-2018 Will Winder

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
package com.willwinder.universalgcodesender.gcode;

import com.willwinder.universalgcodesender.gcode.util.Code;
import com.willwinder.universalgcodesender.gcode.util.Plane;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;

import java.util.EnumSet;

import static com.willwinder.universalgcodesender.gcode.util.Code.*;

/**
 * The current state of a gcode program.
 *
 * @author wwinder
 */
public class GcodeState {

    // Current state
    // group 1
    public Code currentMotionMode = null;

    // group 2
    public Plane plane;

    // group 3
    public boolean inAbsoluteMode = true;
    public Code distanceMode = G90;

    // group 4
    public boolean inAbsoluteIJKMode = false;
    public Code arcDistanceMode = G91_1;

    // group 5
    public Code feedMode = G94;
    public double feedRate = 0;

    // group 6
    public boolean isMetric = true;
    public Code units = G21;

    // group 12
    public Code offset = G54;

    // spindle state
    public Code spindle = M5;

    // coolant state
    public Code coolant = M9;

    // Misc
    public double spindleSpeed = 0;
    public Position currentPoint = null;
    public int commandNumber = 0;

    /**
     * Axes whose position in the active work coordinate system is not known, even though {@link #currentPoint}
     * may hold a value for them. Set by motion in machine coordinates (G53), go-home (G28/G30), probing (G38.x),
     * canned cycles, coordinate frame changes (G92, G10, switching G54-G59.3), tool length offset changes
     * (G43, G43.1, G49), tool changes (M6), program pauses (M0, M1, M60) and relative moves from an unknown position.
     * An axis is known again after the next absolute (G90) work-coordinate move that names it.
     * currentPoint is left as it was so the visualizer is unchanged; processors that rewrite coordinates must use
     * {@link #isPositionKnown(Axis)}.
     */
    public EnumSet<Axis> unknownAxes = EnumSet.noneOf(Axis.class);

    /**
     * True once the program has moved (G0-G3). Used to tell a program selecting its work coordinate system
     * from a program switching to another one.
     */
    public boolean motionStarted = false;

    /**
     * True after the program changed its work coordinate frame: G92, G92.x, G10, or a switch to another work
     * coordinate system after motion has started. Positions after this are in a different frame than before.
     */
    public boolean workFrameChanged = false;

    public GcodeState() {
        // GRBL initial state: [G0 G54 G17 G21 G90 G94 M0 M5 M9 T0 F0. S0.]
        this.currentMotionMode = G0;
        // TODO: Add WCS
        this.plane = Plane.XY;

        // TODO: Feed mode
        this.feedRate = 0;
        this.spindleSpeed = 0;

        this.currentPoint = new Position(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Units.MM);
    }

    public GcodeState copy() {
        GcodeState ret = new GcodeState();
        ret.currentMotionMode = currentMotionMode;
        ret.plane = plane;

        ret.inAbsoluteMode = inAbsoluteMode;
        ret.distanceMode = distanceMode;

        ret.inAbsoluteIJKMode = inAbsoluteIJKMode;
        ret.arcDistanceMode = arcDistanceMode;

        ret.feedMode = feedMode;

        ret.isMetric = isMetric;
        ret.units = units;

        ret.feedRate = feedRate;
        ret.spindleSpeed = spindleSpeed;

        ret.offset = offset;

        ret.spindle = spindle;

        ret.coolant = coolant;

        if (currentPoint != null) {
            ret.currentPoint = new Position(currentPoint.x, currentPoint.y, currentPoint.z, currentPoint.a, currentPoint.b, currentPoint.c, getUnits());
        }
        ret.commandNumber = commandNumber;
        ret.unknownAxes = EnumSet.copyOf(unknownAxes);
        ret.motionStarted = motionStarted;
        ret.workFrameChanged = workFrameChanged;
        return ret;
    }

    /**
     * @return true if the position of the axis in the active work coordinate system is known
     */
    public boolean isPositionKnown(Axis axis) {
        return currentPoint != null && !Double.isNaN(currentPoint.get(axis)) && !unknownAxes.contains(axis);
    }

    /**
     * @return true if the X, Y and Z positions in the active work coordinate system are known
     */
    public boolean isPositionKnownXYZ() {
        return isPositionKnown(Axis.X) && isPositionKnown(Axis.Y) && isPositionKnown(Axis.Z);
    }

    /**
     * Generate gcode to initialize spindle, coolant, and speeds.
     * @return a string of valid gcode like "F300.0S10000.0M3"
     */
    public String toAccessoriesCode() {
        StringBuilder result = new StringBuilder();

        if (spindle != M5) {
            result.append(spindle.toString());
        }
        result.append("S").append(this.spindleSpeed);

        if (coolant != M9) {
            result.append(coolant.toString());
        }

        result.append("F").append(this.feedRate);

        return result.toString();
    }

    /**
     * Generates gcode for the current state of the machine. G-Codes only.
     * @return a string of valid gcode like "G20G91G90.1G93G58G17.1"
     */
    public String machineStateCode() {
        StringBuilder result = new StringBuilder();
        result.append(this.units);
        result.append(this.distanceMode);
        result.append(this.arcDistanceMode);
        result.append(this.feedMode);
        result.append(this.offset);
        result.append(this.plane.code);
        return result.toString();
    }

    /**
     * Returns the states current units
     *
     * @return the units
     */
    public Units getUnits() {
        return UnitUtils.Units.getUnits(units);
    }

    @Override
    public String toString() {
      String pattern = "metric: %b, motionMode: %s, plane: %s, absoluteMode: %b, ijkMode: %b, feed: %f, spindle speed: %f, spindle state: %s, coolant state: %s, point: %s";
      return String.format(pattern,
              isMetric, currentMotionMode, plane, inAbsoluteMode, inAbsoluteIJKMode, feedRate, spindleSpeed, spindle, coolant, currentPoint);

    }
}
