package com.willwinder.ugs.platform.surfacescanner.renderable;

import com.willwinder.ugs.nbm.visualizer.shared.RenderableBounds;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class AutoLevelPreviewTest {
    private static final double DELTA = 0.0001;

    @Test
    public void boundsOfShouldBeTheAreaOfTheProbePointsInMillimeters() {
        // The corners of a scan in inches, the probe points start at the same height
        List<Position> positions = List.of(
                new Position(0.5, 1.0, 0.1, Units.INCH),
                new Position(8.5, 1.0, 0.1, Units.INCH),
                new Position(0.5, 12.0, 0.1, Units.INCH),
                new Position(8.5, 12.0, 0.1, Units.INCH));

        RenderableBounds bounds = AutoLevelPreview.boundsOf(positions);

        assertPosition(12.7, 25.4, 2.54, bounds.min());
        assertPosition(215.9, 304.8, 2.54, bounds.max());
    }

    @Test
    public void boundsOfShouldWidenASinglePoint() {
        RenderableBounds bounds = AutoLevelPreview.boundsOf(List.of(new Position(0, 0, 0, Units.MM)));

        assertPosition(-10, -10, 0, bounds.min());
        assertPosition(10, 10, 0, bounds.max());
    }

    @Test
    public void boundsOfShouldWidenASingleRow() {
        RenderableBounds bounds = AutoLevelPreview.boundsOf(List.of(
                new Position(0, 5, 0, Units.MM),
                new Position(100, 5, 0, Units.MM)));

        assertPosition(0, -5, 0, bounds.min());
        assertPosition(100, 15, 0, bounds.max());
    }

    @Test
    public void boundsOfShouldIgnoreUnknownValues() {
        RenderableBounds bounds = AutoLevelPreview.boundsOf(List.of(
                new Position(0, 0, Double.NaN, Units.MM),
                new Position(Double.NaN, 500, 0, Units.MM),
                new Position(100, Double.POSITIVE_INFINITY, 0, Units.MM),
                new Position(100, 50, 2, Units.MM)));

        assertPosition(0, 0, 0, bounds.min());
        assertPosition(100, 50, 2, bounds.max());
    }

    @Test
    public void boundsOfShouldBeNullWithoutKnownPositions() {
        assertNull(AutoLevelPreview.boundsOf(List.of()));
        assertNull(AutoLevelPreview.boundsOf(List.of(new Position(Double.NaN, 0, 0, Units.MM))));
    }

    private static void assertPosition(double x, double y, double z, Position position) {
        assertEquals(Units.MM, position.getUnits());
        assertEquals(x, position.x, DELTA);
        assertEquals(y, position.y, DELTA);
        assertEquals(z, position.z, DELTA);
    }
}
