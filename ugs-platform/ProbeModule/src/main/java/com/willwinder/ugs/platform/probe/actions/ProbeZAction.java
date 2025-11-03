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
package com.willwinder.ugs.platform.probe.actions;

import com.willwinder.ugs.nbp.lib.services.LocalizingService;
import com.willwinder.ugs.platform.probe.ProbeParameters;
import com.willwinder.ugs.platform.probe.ProbeService;
import com.willwinder.ugs.platform.probe.ProbeSettings;
import com.willwinder.ugs.platform.probe.renderable.ProbePreviewManager;
import com.willwinder.universalgcodesender.i18n.Localization;
import org.openide.awt.ActionID;
import org.openide.awt.ActionReference;
import org.openide.awt.ActionReferences;
import org.openide.awt.ActionRegistration;
import org.openide.util.Lookup;
import org.openide.DialogDisplayer;
import org.openide.NotifyDescriptor;
import org.openide.util.ImageUtilities;

@ActionID(
        category = LocalizingService.CATEGORY_MACHINE,
        id = "com.willwinder.ugs.platform.probe.actions.ProbeZAction")
@ActionRegistration(
        iconBase = ProbeZAction.BASE_ICON,
        displayName = "Probe and zero Z",
        lazy = false)
@ActionReferences({
        @ActionReference(
                path = LocalizingService.MENU_MACHINE_PROBE,
                position = 10)
})
public class ProbeZAction extends AbstractProbeAction {

    public static final String BASE_ICON = "com/willwinder/ugs/platform/probe/icons/zprobe.svg";
    public static final String BASE_ICON_LARGE = "com/willwinder/ugs/platform/probe/icons/zprobe24.svg";

    public ProbeZAction() {
        putValue("iconBase", BASE_ICON);
        putValue(SMALL_ICON, ImageUtilities.loadImageIcon(BASE_ICON, false));
        putValue(LARGE_ICON_KEY, ImageUtilities.loadImageIcon(BASE_ICON_LARGE, false));
        putValue("menuText", Localization.getString("probe.action.z"));
        putValue(NAME, Localization.getString("probe.action.z"));
    }

@Override
public void performProbeAction() {
    ProbeService probeService = Lookup.getDefault().lookup(ProbeService.class);

    // Get current position
    double z;
    try {
        z = getBackend().getWorkPosition().getZ();
    } catch (Exception ex) {
        DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
            "Could not read current Z position. Aborting probe.",
            NotifyDescriptor.ERROR_MESSAGE));
        return;
    }

    // Read probe range settings
    double zMin = ProbeSettings.getzDistance();
    double zMax = 0.0;

    if (z < zMin) {
        DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
            String.format("Current Z (%.3f) is below Z-Min (%.3f). Aborting probe.", z, zMin),
            NotifyDescriptor.ERROR_MESSAGE));
        return;
    }

    if (z > zMax) {
        DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
            String.format("Current Z (%.3f) is above Z-Max (%.3f). Aborting probe.", z, zMax),
            NotifyDescriptor.WARNING_MESSAGE));
        return;
    }

    if ((zMax - zMin) < 0.050) {
        DialogDisplayer.getDefault().notify(new NotifyDescriptor.Message(
            String.format("Z probe range is very small (%.3f in). Aborting probe.", zMax - zMin),
            NotifyDescriptor.WARNING_MESSAGE));
        return;
    }

    // Build probe config
    ProbeParameters pc = new ProbeParameters(
        ProbeSettings.getSettingsProbeDiameter(),
        getBackend().getMachinePosition(),
        0., 0., ProbeSettings.getzDistance(),
        0., 0., ProbeSettings.getzOffset(),
        0.0,
        ProbeSettings.getSettingsFastFindRate(),
        ProbeSettings.getSettingsSlowMeasureRate(),
        ProbeSettings.getSettingsRetractAmount(),
        ProbeSettings.getSettingsDelayAfterRetract(),
        getBackend().getSettings().getPreferredUnits(),
        ProbeSettings.getSettingsWorkCoordinate()
    );

    // Update preview
    ProbePreviewManager probePreviewManager = Lookup.getDefault().lookup(ProbePreviewManager.class);
    probePreviewManager.updateContext(pc, getBackend().getWorkPosition(), getBackend().getMachinePosition());

    // Run probe
    probeService.performZProbe(pc);
}

    @Override
    public String getProbeConfirmationText() {
        return Localization.getString("probe.action.z.confirmation");
    }
}
