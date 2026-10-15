package com.willwinder.ugs.platform.surfacescanner.actions;

import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class OpenScannedSurfaceActionTest {
    private static final double DELTA = 0.00001;

    @Test
    public void parseUnitsShouldReadTheUnitsWrittenWhenSaving() {
        assertEquals(Units.INCH, OpenScannedSurfaceAction.parseUnits("INCH"));
        assertEquals(Units.MM, OpenScannedSurfaceAction.parseUnits("MM"));
        assertEquals(Units.MM, OpenScannedSurfaceAction.parseUnits("mm"));
        assertNull(OpenScannedSurfaceAction.parseUnits("feet"));
    }

    @Test
    public void toUnitsShouldConvertAScanSavedInOtherUnits() {
        // Read as inches as the program is in inches, but the file was saved in millimeters
        List<Position> positions = List.of(new Position(25.4, 50.8, -0.254, Units.INCH));

        List<Position> converted = OpenScannedSurfaceAction.toUnits(positions, Units.MM, Units.INCH);

        Position position = converted.get(0);
        assertEquals(Units.INCH, position.getUnits());
        assertEquals(1.0, position.getX(), DELTA);
        assertEquals(2.0, position.getY(), DELTA);
        assertEquals(-0.01, position.getZ(), DELTA);
    }

    @Test
    public void toUnitsShouldKeepTheValuesWhenTheUnitsAreTheSameOrUnknown() {
        List<Position> positions = List.of(new Position(1.0, 2.0, -0.01, Units.INCH));

        assertSame(positions, OpenScannedSurfaceAction.toUnits(positions, Units.INCH, Units.INCH));
        assertSame(positions, OpenScannedSurfaceAction.toUnits(positions, null, Units.INCH));
    }

    @Test
    public void validatePositionsShouldAcceptACompleteScan() {
        List<Position> positions = List.of(
                new Position(0, 0, 0, Units.INCH),
                new Position(1, 0, -0.01, Units.INCH),
                new Position(0, 1, 0.01, Units.INCH),
                new Position(1, 1, 0, Units.INCH));

        OpenScannedSurfaceAction.validatePositions(positions, 2, 2);
        OpenScannedSurfaceAction.validatePositions(positions, 0, 0);
    }

    @Test
    public void validatePositionsShouldRejectAnEmptyFile() {
        IllegalArgumentException exception = assertThrows(IllegalArgumentException.class,
                () -> OpenScannedSurfaceAction.validatePositions(List.of(), 0, 0));
        assertTrue(exception.getMessage().contains("no scan points"));
    }

    @Test
    public void validatePositionsShouldRejectAFileThatIsCutShort() {
        List<Position> positions = List.of(
                new Position(0, 0, 0, Units.INCH),
                new Position(1, 0, -0.01, Units.INCH),
                new Position(0, 1, 0.01, Units.INCH));

        IllegalArgumentException exception = assertThrows(IllegalArgumentException.class,
                () -> OpenScannedSurfaceAction.validatePositions(positions, 2, 2));
        assertTrue(exception.getMessage().contains("3 scan points"));
    }

    @Test
    public void validatePositionsShouldRejectPointsThatAreNotNumbers() {
        List<Position> positions = List.of(new Position(0, 0, Double.NaN, Units.INCH));

        assertThrows(IllegalArgumentException.class, () -> OpenScannedSurfaceAction.validatePositions(positions, 0, 0));
    }
}
