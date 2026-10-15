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

import com.willwinder.universalgcodesender.gcode.GcodeParser.GcodeMeta;
import com.willwinder.universalgcodesender.gcode.GcodePreprocessorUtils;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import com.willwinder.universalgcodesender.gcode.util.Code;

import java.util.List;

/**
 * Rules shared by the processors that rewrite the coordinates of a move (line splitting, arc expansion, mesh
 * leveling), so they never turn a move into a different move.
 */
final class CoordinateRewriteGuard {

    private CoordinateRewriteGuard() {
    }

    /**
     * A line must be passed through unchanged if its axis words are not a target in the active work coordinate
     * system (G53, G28, G30, G38.x), if it changes that system or the tool offset (G10, G92, G43, G49, a different
     * G54-G59.3), if it has a G-code the parser does not model, if it moves a rotary axis, or if it changes tool.
     *
     * @param command  the original line
     * @param commands the parsed line
     * @param state    the state before the line
     */
    static boolean mustPassThrough(String command, List<GcodeMeta> commands, GcodeState state) {
        for (GcodeMeta meta : commands) {
            if (meta.code.isNonWorkCoordinateCode()) {
                return true;
            }
            if (meta.code.getType() == Code.ModalGroup.WCS && meta.code != state.offset) {
                return true;
            }
        }

        List<String> words = GcodePreprocessorUtils.splitCommand(command);
        if (GcodePreprocessorUtils.getGCodes(words).contains(Code.UNKNOWN)) {
            return true;
        }

        for (String word : words) {
            char c = Character.toUpperCase(word.charAt(0));
            if (c == 'A' || c == 'B' || c == 'C') {
                return true;
            }
            // A tool change runs before the motion on its line, so the move doesn't start where the state says
            if (c == 'M' && isToolChange(word)) {
                return true;
            }
        }
        return false;
    }

    private static boolean isToolChange(String mWord) {
        try {
            return Double.parseDouble(mWord.substring(1)) == 6;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    /**
     * Splits the non-motion part of a line that is replaced by several moves into the words that run before the
     * moves and the program stops (M0, M1, M2, M30, M60) that must run after them.
     *
     * @return {before, after}, either may be empty
     */
    static String[] splitProgramStops(String remainder) {
        StringBuilder before = new StringBuilder();
        StringBuilder after = new StringBuilder();
        for (String word : GcodePreprocessorUtils.splitCommand(remainder)) {
            Code code = Character.toUpperCase(word.charAt(0)) == 'M' ? Code.lookupCode(word) : null;
            if (code != null && code.getType() == Code.ModalGroup.Stopping) {
                after.append(word);
            } else {
                before.append(word);
            }
        }
        return new String[]{before.toString(), after.toString()};
    }
}
