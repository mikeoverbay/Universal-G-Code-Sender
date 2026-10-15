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
package com.willwinder.universalgcodesender.gcode.processors;

import com.willwinder.universalgcodesender.gcode.GcodeParser;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import com.willwinder.universalgcodesender.gcode.util.Code;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserException;
import com.willwinder.universalgcodesender.gcode.util.GcodeParserUtils;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.Position;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static com.willwinder.universalgcodesender.model.UnitUtils.Units.INCH;
import static com.willwinder.universalgcodesender.model.UnitUtils.Units.MM;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.Assert.assertEquals;

/**
 * The auto leveler chain (arc expander, line splitter, mesh leveler) must never turn a move into a different
 * move: machine coordinate moves, moves from an unknown position and moves after a change of work coordinate
 * frame are passed through unchanged.
 */
public class AutoLevelSafetyTest {

    /**
     * 3x3 inch grid X 0..4, Y 0..4
     */
    private static Position[][] grid() {
        double[][] z = {{0.000, 0.010, 0.030}, {-0.010, 0.005, 0.020}, {-0.020, 0.000, 0.015}};
        Position[][] g = new Position[3][3];
        for (int xi = 0; xi < 3; xi++) {
            for (int yi = 0; yi < 3; yi++) {
                g[xi][yi] = new Position(xi * 2.0, yi * 2.0, z[xi][yi], INCH);
            }
        }
        return g;
    }

    private static ArcExpander strictArcExpander(double length) {
        return new ArcExpander(true, length, GcodePreprocessorUtils.getDecimalFormatter(), true);
    }

    private static CommandProcessorList chain() {
        CommandProcessorList result = new CommandProcessorList();
        result.add(strictArcExpander(0.1));
        result.add(new LineSplitter(0.5));
        result.add(new MeshLeveler(0.0, grid()));
        return result;
    }

    /**
     * Same loop as GcodeParserUtils.preprocessAndWrite, used by the export
     */
    private static List<String> export(CommandProcessor processor, String... lines) throws GcodeParserException {
        GcodeParser gcp = new GcodeParser();
        gcp.addCommandProcessor(processor);
        List<String> out = new ArrayList<>();
        for (String line : lines) {
            out.addAll(gcp.preprocessCommand(line, gcp.getCurrentState()));
            gcp.addCommand(line);
        }
        return out;
    }

    private static List<String> export(String... lines) throws GcodeParserException {
        return export(chain(), lines);
    }

    private static GcodeState parse(String... lines) throws GcodeParserException {
        GcodeParser gcp = new GcodeParser();
        for (String line : lines) {
            gcp.addCommand(line);
        }
        return gcp.getCurrentState();
    }

    private static GcodeState inchState(double x, double y, double z) {
        GcodeState state = new GcodeState();
        state.units = Code.G20;
        state.isMetric = false;
        state.currentPoint = new Position(x, y, z, INCH);
        state.inAbsoluteMode = true;
        return state;
    }

    // ---- G53 must not be turned into a work coordinate move -----------------------------------------------------

    @Test
    public void lineSplitterPassesLongG53MoveThroughUnchanged() throws Exception {
        LineSplitter splitter = new LineSplitter(0.25);
        GcodeState state = inchState(1, 1, 0.6);
        assertThat(splitter.processCommand("G53 G0 Z0", state)).containsExactly("G53 G0 Z0");
        assertThat(splitter.processCommand("G0 G53 Z-0.5", state)).containsExactly("G0 G53 Z-0.5");
        assertThat(splitter.processCommand("G53 X-10 Y-5", state)).containsExactly("G53 X-10 Y-5");
    }

    @Test
    public void meshLevelerEmitsG53LineOnceAndUnchanged() throws Exception {
        MeshLeveler leveler = new MeshLeveler(0, grid());
        assertThat(leveler.processCommand("G53 G0 Z0", inchState(0.2, 3.8, 0.3))).containsExactly("G53 G0 Z0");
        assertThat(leveler.processCommand("G53 G0 X3.2 Y1", inchState(3, 1, 0.1))).containsExactly("G53 G0 X3.2 Y1");
    }

    // ---- the first move after G53 must not become a 3-axis move -------------------------------------------------

    @Test
    public void parserMarksG53AxesUnknownButKeepsCurrentPoint() throws Exception {
        GcodeState state = parse("G20 G90", "G0 X1 Y1 Z0.6", "G53 G0 Z0");
        assertThat(state.isPositionKnown(Axis.Z)).isFalse();
        assertThat(state.isPositionKnown(Axis.X)).isTrue();
        assertThat(state.isPositionKnown(Axis.Y)).isTrue();
        assertThat(state.currentPoint.z).isEqualTo(0.0); // visualizer behaviour unchanged

        state = parse("G20 G90", "G0 X1 Y1 Z0.6", "G53 G0 Z0", "G0 Z0.2");
        assertThat(state.isPositionKnownXYZ()).isTrue();
    }

    @Test
    public void firstXYMoveAfterG53IsNotGivenAZWord() throws Exception {
        assertThat(export("G20 G90 G17", "G53 G0 Z0", "G0 X1 Y1", "G0 Z0.1", "G1 Z-0.01 F10", "G1 X3 Y1 F30",
                "G0 Z0.1", "G53 G0 Z0", "G0 X0 Y0", "M30"))
                .containsExactly("G20 G90 G17", "G53 G0 Z0", "G0 X1 Y1", "G0 Z0.1013", "G1 Z-0.0088 F10", "F30",
                        "G1X1.5Y1Z-0.0106", "G1X2Y1Z-0.0125", "G1X2.5Y1Z-0.0144", "G1X3Y1Z-0.0163",
                        "G0 Z0.0938", "G53 G0 Z0", "G0 X0 Y0", "M30");
    }

    @Test
    public void relativeMoveDoesNotMakeAnUnknownAxisKnown() throws Exception {
        GcodeState state = parse("G20 G90", "G0 X1 Y1 Z0.1", "G53 G0 Z0", "G91 G0 Z-0.5", "G90");
        assertThat(state.isPositionKnown(Axis.Z)).isFalse();
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.1", "G53 G0 Z0", "G91 G0 Z-0.5", "G90", "G0 X2 Y2"))
                .endsWith("G91 G0 Z-0.5", "G90", "G0 X2 Y2");
    }

    @Test
    public void programStartingInRelativeModeIsNotLeveledFromAGuessedStart() throws Exception {
        String[] program = {"G20 G91 G17", "G0 X1 Y1", "G1 Z-0.25 F20", "G1 X2", "G90", "G1 X3 Y1"};
        // The relative move is split (only distances are written) but nothing is leveled, and the first absolute
        // move is not given a Z calculated from an assumed start at zero
        assertThat(export(program)).containsExactly("G20 G91 G17", "G0 X1 Y1", "G1 Z-0.25 F20",
                "G1X0.5Y0Z0", "G1X0.5Y0Z0", "G1X0.5Y0Z0", "G1X0.5Y0Z0", "G90", "G1 X3 Y1");
        assertThat(parse(program).unknownAxes).contains(Axis.Z);
    }

    // ---- other codes that leave the work position unknown -------------------------------------------------------

    @Test
    public void goHomeProbeAndOffsetCodesMarkAxesUnknown() throws Exception {
        assertThat(parse("G0 X1 Y1 Z1", "G28 G91 Z0").unknownAxes).containsExactly(Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "G28").unknownAxes).containsExactlyInAnyOrder(Axis.values());
        assertThat(parse("G0 X1 Y1 Z1", "G30").unknownAxes).containsExactlyInAnyOrder(Axis.values());
        assertThat(parse("G0 X1 Y1 Z1", "G38.2 Z-1 F5").unknownAxes).containsExactly(Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "G92 Z0").unknownAxes).containsExactly(Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "G92.1").unknownAxes).containsExactlyInAnyOrder(Axis.values());
        assertThat(parse("G0 X1 Y1 Z1", "G10 L20 P1 Z0.5").unknownAxes).containsExactly(Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "G55").unknownAxes).containsExactlyInAnyOrder(Axis.values());
        assertThat(parse("G0 X1 Y1 Z1", "G54").unknownAxes).isEmpty(); // same WCS as the default
    }

    @Test
    public void toolChangeAndPauseMakeThePositionUnknown() throws Exception {
        assertThat(parse("G0 X1 Y1 Z1", "T2 M6").unknownAxes).contains(Axis.X, Axis.Y, Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "M0").unknownAxes).contains(Axis.X, Axis.Y, Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "M1").unknownAxes).contains(Axis.X, Axis.Y, Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "G0 Z1.5 M0").unknownAxes).contains(Axis.X, Axis.Y, Axis.Z);
        assertThat(parse("G0 X1 Y1 Z1", "M5").unknownAxes).isEmpty();

        // After a tool change the next XY rapid is not given a Z word from the old tool's height
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.6", "G1 Z-0.01 F10", "G0 Z0.6", "M5", "T2 M6", "G0 X3 Y3"))
                .endsWith("M5", "T2 M6", "G0 X3 Y3");
    }

    @Test
    public void cannedCyclesLeaveThePositionUnknown() throws Exception {
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.2", "G99 G81 X1 Y1 Z-0.2 R0.1 F10", "X3 Y1", "G80", "G0 X3 Y3"))
                .endsWith("G99 G81 X1 Y1 Z-0.2 R0.1 F10", "X3 Y1", "G80", "G0 X3 Y3");
    }

    @Test
    public void g43_1ZWordIsNotAMove() throws Exception {
        GcodeState start = inchState(1, 1, 0.2);
        List<GcodeParser.GcodeMeta> metas = GcodeParserUtils.processCommand("G43.1 Z-1.234", 0, start);
        assertThat(metas).extracting(m -> m.code).containsExactly(Code.G43_1);
        assertThat(metas.get(0).state.currentPoint.z).isEqualTo(0.2);
        assertThat(metas.get(0).state.isPositionKnown(Axis.Z)).isFalse();
        assertThat(new LineSplitter(0.25).processCommand("G43.1 Z-1.234", start)).containsExactly("G43.1 Z-1.234");
    }

    // ---- a change of work coordinate frame stops leveling -------------------------------------------------------

    @Test
    public void wcsSwitchAfterMotionStopsLeveling() throws Exception {
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.2", "G1 Z-0.01 F10", "G0 Z0.2", "G55", "G0 X1 Y1", "G0 Z0.2", "G1 Z-0.01 F10"))
                .endsWith("G55", "G0 X1 Y1", "G0 Z0.2", "G1 Z-0.01 F10");
    }

    @Test
    public void g92AndG10StopLeveling() throws Exception {
        assertThat(export("G20 G90 G17", "G92 X0 Y0 Z0", "G0 X1 Y1 Z0.2", "G1 Z-0.01 F10"))
                .containsExactly("G20 G90 G17", "G92 X0 Y0 Z0", "G0 X1 Y1 Z0.2", "G1 Z-0.01 F10");
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.2", "G10 L20 P1 Z0.0625", "G0 Z0.2", "G1 Z-0.01 F10"))
                .endsWith("G10 L20 P1 Z0.0625", "G0 Z0.2", "G1 Z-0.01 F10");
    }

    @Test
    public void selectingTheWcsBeforeTheFirstMoveStillLevels() throws Exception {
        assertThat(export("G20 G90 G17", "G55", "G0 X1 Y1 Z0.2", "G1 Z-0.01 F10"))
                .endsWith("G1 Z-0.0088 F10");
    }

    @Test
    public void repeatingTheActiveWcsOnAMoveIsStillLeveled() throws Exception {
        MeshLeveler leveler = new MeshLeveler(0, grid());
        assertThat(leveler.processCommand("G90 G54 G1 Z-0.05 F10", inchState(1, 1, 0.1)))
                .containsExactly("G90 G54 G1 Z-0.0488 F10");
    }

    // ---- MeshLeveler emits each line once -----------------------------------------------------------------------

    @Test
    public void meshLevelerEmitsLineWithExtraGCodesOnce() throws Exception {
        MeshLeveler leveler = new MeshLeveler(0, grid());
        assertThat(leveler.processCommand("G90 G1 Z-0.01 F10", inchState(1, 1, 0.1))).containsExactly("G90 G1 Z-0.0088 F10");
        // G91 on the line: one relative line, Z is the surface delta
        assertThat(leveler.processCommand("G91 G1 X0.2 F10", inchState(1, 1, 0.1))).containsExactly("G91 G1 X0.2 F10Z-0.0008");
        assertThat(leveler.processCommand("G17 G90 G0 X1.2 Y1.1", inchState(1, 1, 0.1))).containsExactly("G17 G90 G0 X1.2 Y1.1Z0.1012");
    }

    @Test
    public void distanceModeAfterTheMotionWordStillApplies() throws Exception {
        List<GcodeParser.GcodeMeta> metas = GcodeParserUtils.processCommand("G1 X5 G91", 0, inchState(1, 1, 0.1));
        GcodeParser.GcodeMeta g1 = metas.stream().filter(m -> m.code == Code.G1).findFirst().get();
        assertThat(g1.point.point().x).isEqualTo(6.0);
        assertThat(g1.state.inAbsoluteMode).isFalse();
    }

    // ---- rotary axes and program stops --------------------------------------------------------------------------

    @Test
    public void rotaryMovesArePassedThrough() throws Exception {
        GcodeState state = inchState(1, 1, -0.01);
        assertThat(new LineSplitter(0.5).processCommand("G1 X3 A90 F30", state)).containsExactly("G1 X3 A90 F30");
        assertThat(new MeshLeveler(0, grid()).processCommand("G1 X3 A90 F30", state)).containsExactly("G1 X3 A90 F30");
    }

    @Test
    public void programStopRunsAfterTheSplitMove() throws Exception {
        // A pause on the line leaves the position unknown afterwards, so the move is passed through whole
        assertThat(new LineSplitter(0.5).processCommand("G0 Z1.5 M0", inchState(1, 1, -0.1))).containsExactly("G0 Z1.5 M0");

        List<String> result = new LineSplitter(0.5).processCommand("G0 Z1.5 M2", inchState(1, 1, -0.1));
        assertThat(result).hasSizeGreaterThan(2);
        assertThat(result.get(0)).startsWith("G0");
        assertThat(result.get(result.size() - 1)).isEqualTo("M2");

        result = new LineSplitter(0.5).processCommand("G1 X3 F30 M30", inchState(1, 1, -0.1));
        assertThat(result.get(0)).isEqualTo("F30");
        assertThat(result.get(result.size() - 1)).isEqualTo("M30");
    }

    // ---- LineSplitter segment length ----------------------------------------------------------------------------

    @Test
    public void segmentLengthIsConvertedToTheProgramUnits() throws Exception {
        GcodeState mm = new GcodeState();
        mm.currentPoint = new Position(0, 0, 0, MM);
        // 10 mm is shorter than 0.5 inch, so not split
        assertThat(new LineSplitter(0.5, INCH).processCommand("G1 X10", mm)).containsExactly("G1 X10");
        // 0.5 mm segments
        assertThat(new LineSplitter(0.5).processCommand("G1 X10", mm)).hasSize(20);
        // a zero length never splits (and never hangs)
        assertThat(new LineSplitter(0).processCommand("G1 X10", mm)).containsExactly("G1 X10");
    }

    // ---- MeshLeveler mesh lookup --------------------------------------------------------------------------------

    @Test
    public void surfaceHeightIsClampedOutsideTheGrid() {
        MeshLeveler leveler = new MeshLeveler(0, grid());
        assertEquals(0.0, leveler.surfaceHeightAt(-6, -6), 1e-9);    // corner (0,0)
        assertEquals(-0.02, leveler.surfaceHeightAt(20, -6), 1e-9);  // corner (4,0)
        assertEquals(0.015, leveler.surfaceHeightAt(10, 10), 1e-9);  // corner (4,4)
        assertEquals(-0.02, leveler.surfaceHeightAt(12, 0), 1e-9);   // beyond the X edge at Y=0
    }

    @Test
    public void nonSquareCellsUseTheirOwnStep() {
        // X 0..4 step 1, Y 0..1 step 0.25, dished surface z = -0.12*y*(1-y)
        Position[][] g = new Position[5][5];
        for (int xi = 0; xi < 5; xi++) {
            for (int yi = 0; yi < 5; yi++) {
                double y = yi * 0.25;
                g[xi][yi] = new Position(xi, y, -0.12 * y * (1 - y), INCH);
            }
        }
        MeshLeveler leveler = new MeshLeveler(0, g);
        // y=0.9 lies in the cell 0.75..1.0: linear between -0.0225 and 0
        assertEquals(-0.009, leveler.surfaceHeightAt(2, 0.9), 1e-9);
        assertEquals(-0.03, leveler.surfaceHeightAt(2, 0.5), 1e-9);
    }

    @Test
    public void commentsAreNotRewritten() throws Exception {
        MeshLeveler leveler = new MeshLeveler(0, grid());
        assertThat(leveler.processCommand("G1 X3.2 Y1 (short Z1 cut)", inchState(3, 1, -0.01))).containsExactly("G1X3.2Y1Z-0.017");
        assertThat(leveler.processCommand("G1 X1.3 Y1.1 ; comment", inchState(1, 1, -0.01))).containsExactly("G1X1.3Y1.1Z-0.0092");
        assertThat(leveler.processCommand("(c1) G1 X1.2 (c2) Y1.1", inchState(1, 1, -0.01))).containsExactly("G1X1.2Y1.1Z-0.0089");
    }

    // ---- ArcExpander --------------------------------------------------------------------------------------------

    @Test
    public void arcFromUnknownStartIsKeptNotDropped() throws Exception {
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.1", "G1 Z-0.01 F10", "G53 G0 X-10 Y-5", "G2 X3 Y1 I1 J0 F30"))
                .endsWith("G53 G0 X-10 Y-5", "G2 X3 Y1 I1 J0 F30");
    }

    @Test
    public void arcInA2DProgramIsStillExpandedAfterG28() throws Exception {
        List<String> out = export(strictArcExpander(1), "G21 G90 G17", "G28", "G0 X10 Y10", "G2 X30 Y10 I10 J0 F300");
        assertThat(out).startsWith("G21 G90 G17", "G28", "G0 X10 Y10");
        List<String> arc = out.subList(3, out.size());
        assertThat(arc).hasSizeGreaterThan(10);
        assertThat(arc).noneMatch(line -> line.contains("Z"));
        assertThat(arc.get(arc.size() - 1)).isEqualTo("G1X30Y10");
    }

    @Test
    public void helicalArcWithUnknownZIsPassedThrough() throws Exception {
        assertThat(export(strictArcExpander(1), "G21 G90 G17", "G28", "G0 X10 Y10", "G2 X30 Y10 Z-1 I10 J0 F300"))
                .endsWith("G2 X30 Y10 Z-1 I10 J0 F300");
    }

    @Test
    public void arcContinuationPassedThroughAfterAnExpandedArcKeepsItsArcCode() throws Exception {
        // The first arc is expanded into G1 lines, the second continues G2 modally but moves an unknown Z
        List<String> out = export(strictArcExpander(1), "G21 G90 G17", "G0 X20 Y20 Z5", "G53 G0 Z-2",
                "G2 X40 Y20 I10 J0", "X20 Y20 Z3 I-10 J0");
        assertThat(out.get(out.size() - 1)).isEqualTo("G2X20 Y20 Z3 I-10 J0");
    }

    @Test
    public void arcAfterToolChangeIsStillExpandedForTransforms() throws Exception {
        // Translate, rotate and mirror embed the default ArcExpander, which expands from the parser position
        List<String> out = export(new ArcExpander(true, 1), "G21 G90", "G0 X0 Y0 Z5", "G1 Z-1 F300", "G1 X20", "T2 M6",
                "G17 G2 X40 Y0 I10 J0");
        assertThat(out.get(out.size() - 1)).isEqualTo("G1X40Y0Z-1");
        assertThat(out).hasSizeGreaterThan(20);
        assertThat(out).noneMatch(line -> line.contains("G2 X40"));
    }

    // ---- review findings ----------------------------------------------------------------------------------------

    @Test
    public void g53RetractBeforeSelectingTheWcsStillLevels() throws Exception {
        // Fusion 360 header with work offset 2
        assertThat(export("G90 G94 G17 G20", "G53 G0 Z0", "T1 M6", "S10000 M3", "G55", "G0 X1 Y1", "G0 Z0.1",
                "G1 Z-0.01 F10"))
                .endsWith("G55", "G0 X1 Y1", "G0 Z0.1013", "G1 Z-0.0088 F10");
    }

    @Test
    public void toolTableAndInactiveWorkOffsetDoNotStopLeveling() throws Exception {
        assertThat(parse("G0 X1 Y1 Z1", "G10 L1 P2 Z-1.5").workFrameChanged).isFalse();
        assertThat(parse("G0 X1 Y1 Z1", "G10 L2 P2 Z-1.5").workFrameChanged).isFalse(); // G55 while G54 is active
        assertThat(parse("G0 X1 Y1 Z1", "G10 L2 P1 Z-1.5").workFrameChanged).isTrue();
        assertThat(parse("G0 X1 Y1 Z1", "G10 L20 P0 Z0").workFrameChanged).isTrue();
        assertThat(export("G20 G90 G17", "G0 X1 Y1 Z0.2", "G10 L1 P2 Z-1.5", "G1 Z-0.01 F10"))
                .endsWith("G10 L1 P2 Z-1.5", "G1 Z-0.0088 F10");
    }

    @Test
    public void toolChangeOnAMotionLineIsNotSplitFromThePreviousPosition() throws Exception {
        assertThat(new LineSplitter(0.5).processCommand("T2 M6 G0 X3 Y3 Z0.5", inchState(1, 1, 0.2)))
                .containsExactly("T2 M6 G0 X3 Y3 Z0.5");
    }

    @Test
    public void relativeMovesAreSplit() throws Exception {
        // Relative segments only depend on the distance, so G91 moves are split even when the absolute position
        // is unknown, here after a tool change
        assertThat(export(new LineSplitter(0.5), "G21 G90", "G0 X0 Y0 Z0", "T2 M6", "G91", "G1 X2 F100"))
                .containsExactly("G21 G90", "G0 X0 Y0 Z0", "T2 M6", "G91", "F100",
                        "G1X0.5Y0Z0", "G1X0.5Y0Z0", "G1X0.5Y0Z0", "G1X0.5Y0Z0");
    }

    // ---- unchanged upstream behaviour ---------------------------------------------------------------------------

    @Test
    public void xyMoveAfterKnownZStillGetsLeveledZ() throws Exception {
        // circle_test fixture behaviour: "Z10" then "G0 X13.86 Y29.12" gets Z10 (flat mesh)
        Position[][] flat = {{new Position(-5, -5, 0, MM), new Position(-5, 35, 0, MM)},
                {new Position(35, -5, 0, MM), new Position(35, 35, 0, MM)}};
        CommandProcessorList processors = new CommandProcessorList();
        processors.add(new LineSplitter(1));
        processors.add(new MeshLeveler(0, flat));
        assertThat(export(processors, "G90 G94", "G21", "G28 G91 Z0", "G90", "Z10", "G54", "G0 X13.86 Y29.12"))
                .endsWith("Z10", "G54", "G0 X13.86 Y29.12Z10");
    }
}
