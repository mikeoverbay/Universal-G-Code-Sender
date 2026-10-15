/*
    Copyright 2026 Will Winder

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
package com.willwinder.ugs.nbm.visualizer.shared;

import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;

import java.util.Collection;

/**
 * The area a renderable draws in, in millimeters
 *
 * @param min the lowest corner
 * @param max the highest corner
 */
public record RenderableBounds(Position min, Position max) {

    /**
     * Returns the area drawn by the enabled renderables that have one
     *
     * @param renderables the renderables
     * @return the combined area or null if none of them have one
     */
    public static RenderableBounds of(Collection<Renderable> renderables) {
        RenderableBounds result = null;
        for (Renderable r : renderables) {
            if (!r.isEnabled()) {
                continue;
            }

            RenderableBounds bounds = r.getBounds().orElse(null);
            result = result == null ? bounds : result.union(bounds);
        }
        return result;
    }

    /**
     * Returns the area covering both this and the other area
     *
     * @param other the other area, may be null
     * @return the combined area
     */
    public RenderableBounds union(RenderableBounds other) {
        if (other == null) {
            return this;
        }

        return new RenderableBounds(
                new Position(Math.min(min.x, other.min.x), Math.min(min.y, other.min.y), Math.min(min.z, other.min.z), Units.MM),
                new Position(Math.max(max.x, other.max.x), Math.max(max.y, other.max.y), Math.max(max.z, other.max.z), Units.MM));
    }
}
