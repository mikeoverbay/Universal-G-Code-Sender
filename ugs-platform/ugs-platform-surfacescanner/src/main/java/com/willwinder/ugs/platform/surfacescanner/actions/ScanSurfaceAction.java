/*
    Copyright 2023 Will Winder

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
import com.willwinder.universalgcodesender.i18n.Localization;
import com.willwinder.universalgcodesender.listeners.UGSEventListener;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.UGSEvent;
import com.willwinder.universalgcodesender.model.events.ControllerStatusEvent;

import org.openide.DialogDisplayer;
import org.openide.NotifyDescriptor;
import org.openide.util.ImageUtilities;

import javax.swing.*;
import java.awt.event.ActionEvent;
import static com.willwinder.ugs.platform.surfacescanner.Utils.shouldEraseProbedData;
import com.willwinder.universalgcodesender.utils.AutoLevelSettings;
import org.openide.util.RequestProcessor;
import javax.swing.SwingUtilities;
import org.openide.DialogDisplayer;
import org.openide.NotifyDescriptor;

public class ScanSurfaceAction extends AbstractAction implements UGSEventListener {

    public static final String ICON_BASE = "com/willwinder/ugs/platform/surfacescanner/icons/scan.svg";
    private final SurfaceScanner surfaceScanner;
    private final BackendAPI backend;
	
	private static final RequestProcessor RP = new RequestProcessor(ScanSurfaceAction.class);
	private static final String STOP_TITLE = "Stop scan";
	// What the button currently shows (EDT only). A click on "Stop scan" must never start a scan,
	// even if the scan ended just before the click was handled.
	private boolean showingStop = false;

    public ScanSurfaceAction(SurfaceScanner surfaceScanner) {
        this.backend = CentralLookup.getDefault().lookup(BackendAPI.class);
        if (this.backend != null) {
            this.backend.addUGSEventListener(this);
        }

        this.surfaceScanner = surfaceScanner;
        // Switch between Scan / Stop as the scan starts and ends
        surfaceScanner.addListener(() -> SwingUtilities.invokeLater(this::updateState));

        setTitle(Localization.getString("autoleveler.panel.scan-surface"));
        putValue("iconBase", ICON_BASE);
        putValue(SMALL_ICON, ImageUtilities.loadImageIcon(ICON_BASE, false));
        putValue(Action.SELECTED_KEY, false);
    }

    private void setTitle(String title) {
        putValue(NAME, title);
        putValue("menuText", title);
        putValue(Action.SHORT_DESCRIPTION, title);
    }

    private void updateState() {
        showingStop = surfaceScanner.isScanning();
        setTitle(showingStop ? STOP_TITLE : Localization.getString("autoleveler.panel.scan-surface"));
        setEnabled(isEnabled());
    }

    @Override
    public boolean isEnabled() {
        // Stop must stay clickable while the machine is moving
        if (surfaceScanner != null && surfaceScanner.isScanning()) {
            return true;
        }
        return backend != null && backend.isConnected() && backend.isIdle();
    }



@Override
public void actionPerformed(ActionEvent e) {
    if (surfaceScanner.isScanning() || showingStop) {
        surfaceScanner.requestStop();
        return;
    }

    if (surfaceScanner.isValid() && !shouldEraseProbedData()) {
        return;
    }
    if (!validateZBeforeScan()) {
        return;
    }

    surfaceScanner.reset();

    // Show Stop right away so a second click before the scan thread starts doesn't queue another scan
    showingStop = true;
    setTitle(STOP_TITLE);

    RP.post(() -> {
        try {
            surfaceScanner.scan();
        } catch (SurfaceScanner.ProbeOverTravelException ex) {
            SwingUtilities.invokeLater(() ->
                DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    ex.getMessage(),
                    NotifyDescriptor.ERROR_MESSAGE
                ))
            );
        } catch (SurfaceScanner.ScanStoppedException ex) {
            SwingUtilities.invokeLater(() ->
                DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    ex.getMessage(),
                    NotifyDescriptor.INFORMATION_MESSAGE
                ))
            );
        } catch (Exception ex) {
            SwingUtilities.invokeLater(() ->
                DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    "Scan failed: " + ex.getMessage(),
                    NotifyDescriptor.ERROR_MESSAGE
                ))
            );
        }
    });
}

    @Override
    public void UGSEvent(UGSEvent evt) {
        if (evt instanceof ControllerStatusEvent) {
            setEnabled(isEnabled());
        }
    }

    private boolean validateZBeforeScan() {
        double zMin, zMax, clearance;
        try {
            AutoLevelSettings settings = backend.getSettings().getAutoLevelSettings();
            zMin = settings.getMinZ();
            zMax = settings.getMaxZ();

        } catch (Exception ex) {
            DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    "Could not read Auto-Level Z settings.", NotifyDescriptor.WARNING_MESSAGE));
            return false;
        }

        if (zMin >= zMax) {
            DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    "Z range is invalid (Z Min ≥ Z Max).", NotifyDescriptor.WARNING_MESSAGE));
            return false;
        }

        double z;
        try {
            z = backend.getWorkPosition().z;
        } catch (Exception ex) {
            DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    "Could not read current Z position. Aborting scan.",
                    NotifyDescriptor.ERROR_MESSAGE));
            return false;
        }

/*         if (z < zMin) {
            DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    String.format("Current Z (%.3f) is below Z-Min (%.3f). This may crash the probe. Aborting scan.", z, zMin),
                    NotifyDescriptor.ERROR_MESSAGE));
            return false;
        }

        if (z > zMax ) {
            DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    String.format("Current Z (%.3f) is above Z-Max + Clearance (%.3f + %.3f). Aborting scan.", z, zMax, 0.0),
                    NotifyDescriptor.WARNING_MESSAGE));
            return false;
        }

        if ((zMax - zMin) < 0.050) {
            DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
                    String.format("Z scan range is very small (%.3f in). Aborting scan.", (zMax - zMin)),
                    NotifyDescriptor.WARNING_MESSAGE));
            return false;
        } */
        return true;
    }
}
