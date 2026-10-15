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
package com.willwinder.ugs.platform.surfacescanner.io;

import org.apache.commons.lang3.StringUtils;

import javax.swing.filechooser.FileFilter;
import java.io.File;

/**
 * A file filter for scan files with the extension .xyz in any case, like scan.xyz or scan.XYZ
 */
public class XyzFileFilter extends FileFilter {
    public static final String EXTENSION = ".xyz";

    @Override
    public boolean accept(File f) {
        // Directories are shown so that they can be navigated
        return f.isDirectory() || StringUtils.endsWithIgnoreCase(f.getName(), EXTENSION);
    }

    @Override
    public String getDescription() {
        return "XYZ scan files (*.xyz)";
    }

    /**
     * Returns the file with the extension .xyz. A new file gets it in lower case, added if the name doesn't end with
     * it. An existing file with it in another case, like scan.XYZ, is kept so that it is the file that is replaced.
     *
     * @param file the file
     * @return the file with the extension .xyz
     */
    public static File withXyzExtension(File file) {
        String name = file.getName();
        if (StringUtils.endsWithIgnoreCase(name, EXTENSION)) {
            if (file.exists()) {
                return file;
            }
            name = name.substring(0, name.length() - EXTENSION.length());
        }
        return new File(file.getParentFile(), name + EXTENSION);
    }
}
