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
package com.willwinder.ugs.nbp.core.services;

import com.willwinder.ugs.nbp.lib.lookup.CentralLookup;
import com.willwinder.universalgcodesender.listeners.ControllerState;
import com.willwinder.universalgcodesender.listeners.MessageType;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.UGSEvent;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.events.ControllerStateEvent;
import com.willwinder.universalgcodesender.utils.Settings;
import org.openide.util.lookup.ServiceProvider;

import java.awt.EventQueue;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * A service that sets the preferred units in the UI to inches when connecting to the controller.
 */
@ServiceProvider(service = InchUnitsOnConnectService.class)
public class InchUnitsOnConnectService {

    private final BackendAPI backend;

    /**
     * Only done once per connection so that the units can still be toggled manually while connected
     */
    private final AtomicBoolean isUnitsPending = new AtomicBoolean(true);

    public InchUnitsOnConnectService() {
        backend = CentralLookup.getDefault().lookup(BackendAPI.class);
        backend.addUGSEventListener(this::onEvent);
    }

    private void onEvent(UGSEvent event) {
        if (!(event instanceof ControllerStateEvent controllerStateEvent)) {
            return;
        }

        ControllerState state = controllerStateEvent.getState();
        if (state == ControllerState.DISCONNECTED) {
            isUnitsPending.set(true);
        } else if (isUnitsPending.compareAndSet(true, false)) {
            // Done on the first state after disconnected, usually connecting, so that the jog settings are rescaled
            // before jogging is allowed. State events come from the connection threads and the settings listeners
            // update the UI directly.
            EventQueue.invokeLater(this::setInchUnits);
        }
    }

    private void setInchUnits() {
        // Changing the units always notifies the listeners and rescales the jog settings, only do it when needed
        Settings settings = backend.getSettings();
        if (settings.getPreferredUnits() != UnitUtils.Units.INCH) {
            backend.dispatchMessage(MessageType.INFO, "*** Setting UI units to inches\n");
            settings.setPreferredUnits(UnitUtils.Units.INCH);
        }
    }
}
