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
package com.willwinder.ugs.platform.surfacescanner.actions;

import com.willwinder.ugs.nbp.lib.lookup.CentralLookup;
import com.willwinder.ugs.platform.surfacescanner.SurfaceScanner;
import com.willwinder.ugs.platform.surfacescanner.Utils; // <-- for extractMetadataValue
import static com.willwinder.ugs.platform.surfacescanner.Utils.getMaxPosition;
import static com.willwinder.ugs.platform.surfacescanner.Utils.getMinPosition;
import static com.willwinder.ugs.platform.surfacescanner.Utils.shouldEraseProbedData;
import com.willwinder.ugs.platform.surfacescanner.io.XyzSurfaceReader;
import com.willwinder.universalgcodesender.i18n.Localization;
import com.willwinder.universalgcodesender.listeners.UGSEventListener;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UGSEvent;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import com.willwinder.universalgcodesender.model.events.ControllerStatusEvent;
import com.willwinder.universalgcodesender.uielements.FileOpenDialog;
import com.willwinder.universalgcodesender.utils.AutoLevelSettings;
import com.willwinder.universalgcodesender.utils.GUIHelpers;
import com.willwinder.universalgcodesender.utils.MathUtils;
import org.apache.commons.lang3.StringUtils;
import org.openide.util.ImageUtilities;
import org.openide.windows.WindowManager;

import javax.swing.AbstractAction;
import javax.swing.Action;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.util.List;
import java.util.Optional;
import java.util.logging.Level;
import java.util.logging.Logger;

public class OpenScannedSurfaceAction extends AbstractAction implements UGSEventListener {

    public static final String ICON_BASE = "com/willwinder/ugs/platform/surfacescanner/icons/open.svg";
    private static final Logger LOGGER = Logger.getLogger(OpenScannedSurfaceAction.class.getSimpleName());

    private final SurfaceScanner surfaceScanner;
    private final BackendAPI backend;

    /**
     * The directory of the last opened scan file, the file dialog starts there the next time
     */
    private String lastDirectory;

    public OpenScannedSurfaceAction(SurfaceScanner surfaceScanner) {
        this.backend = CentralLookup.getDefault().lookup(BackendAPI.class);
        this.backend.addUGSEventListener(this);
        this.surfaceScanner = surfaceScanner;

        String title = Localization.getString("autoleveler.panel.open");
        putValue(NAME, title);
        putValue("menuText", title);
        putValue(Action.SHORT_DESCRIPTION, title);
        putValue("iconBase", ICON_BASE);
        putValue(SMALL_ICON, ImageUtilities.loadImageIcon(ICON_BASE, false));
    }

    @Override
    public boolean isEnabled() {
        return (backend.isConnected() && backend.isIdle()) || !backend.isConnected();
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if (surfaceScanner.isValid() && !shouldEraseProbedData()) {
            return;
        }

        Optional<File> file = chooseFile();
        if (file.isEmpty()) {
            return;
        }
        lastDirectory = file.get().getParent();

        try {
            // 1) Read header lines first to pull out xSamples / ySamples (simple & robust)
            List<String> allLines = Files.readAllLines(file.get().toPath());
            int xSamplesFromHeader = 0;
            int ySamplesFromHeader = 0;
            Units unitsFromHeader = null;
            for (String line : allLines) {
                String trimmed = line.trim();
                if (!trimmed.startsWith("#")) {
                    // stop processing headers, switch to point data
                    continue;
                }
                String xCount = Utils.extractMetadataValue(trimmed, "xSamples");
                if (xCount != null) {
                    try {
                        xSamplesFromHeader = Integer.parseInt(xCount);
                    } catch (NumberFormatException ignored) {
                    }
                }
                String yCount = Utils.extractMetadataValue(trimmed, "ySamples");
                if (yCount != null) {
                    try {
                        ySamplesFromHeader = Integer.parseInt(yCount);
                    } catch (NumberFormatException ignored) {
                    }
                }
                String units = Utils.extractMetadataValue(trimmed, "units");
                if (units != null) {
                    unitsFromHeader = parseUnits(units);
                }
            }

            // 2) Parse positions (the reader should ignore '#' lines)
            // Closed when read, an open file on a network share can stop other programs from replacing it
            XyzSurfaceReader surfaceReader = new XyzSurfaceReader();
            List<Position> positions;
            try (InputStream inputStream = Files.newInputStream(file.get().toPath())) {
                positions = surfaceReader.read(inputStream);
            }

            // Checked before anything is changed, the reader gives the values in the preferred units
            positions = toUnits(positions, unitsFromHeader, backend.getSettings().getPreferredUnits());
            validatePositions(positions, xSamplesFromHeader, ySamplesFromHeader);

            // 3) Update settings (use header counts if present; fallback to distinct counts)
            updateSettings(backend.getSettings().getAutoLevelSettings(), positions, xSamplesFromHeader, ySamplesFromHeader);

            // 4) Fill the scanner’s grid with the loaded Z data
            updatePoints(positions);

        } catch (IOException | RuntimeException ex) {
            // Also invalid lines and grid points missing in the file, which would otherwise fail without a message
            LOGGER.log(Level.WARNING, "Could not read the scanned point file " + file.get(), ex);
            GUIHelpers.displayErrorDialog("Could not read the scanned point file " + file.get().getName() + ": " +
                    StringUtils.defaultIfEmpty(ex.getMessage(), ex.getClass().getSimpleName()));
        }
    }

    /**
     * Returns the units in the header of a saved scan file (# units=INCH)
     *
     * @param units the header value
     * @return the units or null if unknown
     */
    static Units parseUnits(String units) {
        if (Units.INCH.name().equalsIgnoreCase(units)) {
            return Units.INCH;
        } else if (Units.MM.name().equalsIgnoreCase(units)) {
            return Units.MM;
        }
        return null;
    }

    /**
     * Converts the positions, which the reader gives in the preferred units, if the file was saved in other units.
     * Otherwise a scan saved in millimeters would be used as inches or the other way around.
     *
     * @param positions     the positions with the values from the file
     * @param fileUnits     the units the file was saved in or null if unknown
     * @param preferredUnits the units to use
     * @return the positions in the preferred units
     */
    static List<Position> toUnits(List<Position> positions, Units fileUnits, Units preferredUnits) {
        if (fileUnits == null || fileUnits == preferredUnits) {
            return positions;
        }

        return positions.stream()
                .map(p -> new Position(p.getX(), p.getY(), p.getZ(), fileUnits).getPositionIn(preferredUnits))
                .toList();
    }

    /**
     * Checks the points of a scan file before it is used, an empty or cut short file would otherwise replace the
     * scan area settings.
     *
     * @param positions the points in the file
     * @param xSamples  the number of columns in the header or 0 if missing
     * @param ySamples  the number of rows in the header or 0 if missing
     * @throws IllegalArgumentException if the points can't be used
     */
    static void validatePositions(List<Position> positions, int xSamples, int ySamples) {
        if (positions.isEmpty()) {
            throw new IllegalArgumentException("the file has no scan points");
        }

        if (positions.stream().anyMatch(p -> !Double.isFinite(p.getX()) || !Double.isFinite(p.getY()) || !Double.isFinite(p.getZ()))) {
            throw new IllegalArgumentException("the file has a scan point that is not a number");
        }

        if (xSamples > 0 && ySamples > 0 && positions.size() != xSamples * ySamples) {
            throw new IllegalArgumentException(String.format("the file has %d scan points but its header says %d x %d",
                    positions.size(), xSamples, ySamples));
        }
    }

    /**
     * Asks for the scan file, starting in the directory of the last opened scan or else of the loaded gcode file.
     * <p>
     * A new dialog without a file filter is used every time. On Linux the GTK file dialog in Java can fail in its file
     * filter callback when given a file name without a directory. It then never hides the dialog, so the same dialog
     * can't be shown again and the selected file is never returned.
     *
     * @return the selected file or an empty optional if cancelled
     */
    private Optional<File> chooseFile() {
        File gcodeFile = backend.getGcodeFile();
        String directory = StringUtils.firstNonEmpty(lastDirectory,
                gcodeFile != null ? gcodeFile.getParent() : null,
                backend.getSettings().getLastOpenedFilename());

        FileOpenDialog fileOpenDialog = new FileOpenDialog(StringUtils.defaultString(directory));
        fileOpenDialog.centerOn(WindowManager.getDefault().getMainWindow());
        fileOpenDialog.setVisible(true);
        Optional<File> file = fileOpenDialog.getSelectedFile();
        fileOpenDialog.dispose();
        return file;
    }

    @Override
    public void UGSEvent(UGSEvent evt) {
        if (evt instanceof ControllerStatusEvent) {
            setEnabled(isEnabled());
        }
    }

    private void updatePoints(List<Position> positions) {
        Optional<Position> nextProbePoint = surfaceScanner.getNextProbePoint();
        while (nextProbePoint.isPresent()) {
            Position probePoint = nextProbePoint.get();
            Position probedPosition = positions.stream()
                    .filter(p -> MathUtils.isEqual(p.getX(), probePoint.getX(), 0.01)
                    && MathUtils.isEqual(p.getY(), probePoint.getY(), 0.01))
                    .findFirst()
                    .orElseThrow(() -> new IllegalStateException("The supplied file is missing height data for grid point: " + probePoint));

            surfaceScanner.probeEvent(probedPosition);
            nextProbePoint = surfaceScanner.getNextProbePoint();
        }
    }

    // Overload that accepts optional header counts; falls back to inferred counts
    private void updateSettings(AutoLevelSettings autoLevelSettings,
            List<Position> positions,
            int xSamplesFromHeader,
            int ySamplesFromHeader) {

        Position minPosition = getMinPosition(positions);
        Position maxPosition = getMaxPosition(positions);
        autoLevelSettings.setMin(minPosition);
        autoLevelSettings.setMax(maxPosition);

        int xSamples = xSamplesFromHeader > 0
                ? xSamplesFromHeader
                : (int) positions.stream().mapToDouble(Position::getX).distinct().count();

        int ySamples = ySamplesFromHeader > 0
                ? ySamplesFromHeader
                : (int) positions.stream().mapToDouble(Position::getY).distinct().count();

        autoLevelSettings.setXSampleCount(xSamples);
        autoLevelSettings.setYSampleCount(ySamples);

        surfaceScanner.update(minPosition, maxPosition);
        surfaceScanner.reset();
    }
}
