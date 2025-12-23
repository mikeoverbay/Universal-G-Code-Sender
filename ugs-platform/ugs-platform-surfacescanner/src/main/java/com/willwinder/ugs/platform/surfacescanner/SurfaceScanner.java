/*
    Copyright 2017-2023 Will Winder

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
package com.willwinder.ugs.platform.surfacescanner;

import com.google.common.collect.ImmutableList;
import com.willwinder.universalgcodesender.gcode.util.GcodeUtils;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.PartialPosition;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import com.willwinder.universalgcodesender.model.events.ProbeEvent;
import com.willwinder.universalgcodesender.utils.AutoLevelSettings;

import com.willwinder.serialcontrol.serialControl;

import java.util.Deque;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * @author wwinder
 */
public class SurfaceScanner {

    private static final Logger logger = Logger.getLogger(SurfaceScanner.class.getSimpleName());
    private boolean probeReady = false;
    private final BackendAPI backend;
    private final AutoLevelSettings settings;
    private final Set<SurfaceScannerListener> listeners = ConcurrentHashMap.newKeySet();
    private Position[][] probePositionGrid = new Position[0][0];
    private Deque<Position> pendingPositions = new LinkedList<>();
    private Position minXYZ = Position.ZERO;
    private Position maxXYZ = Position.ZERO;
    private Position machineWorkOffset = new Position(Units.MM);
	private Position first = null;

    private final AtomicBoolean isScanning = new AtomicBoolean(false);

    public SurfaceScanner(BackendAPI backend) {
        this.backend = backend;
        this.settings = backend.getSettings().getAutoLevelSettings();
        Position minPosition = new Position(settings.getMinX(), settings.getMinY(), settings.getMinZ(), backend.getSettings().getPreferredUnits());
        Position maxPosition = new Position(settings.getMaxX(), settings.getMaxY(), settings.getMaxZ(), backend.getSettings().getPreferredUnits());
        update(minPosition, maxPosition);
    }

    public boolean isProbeReady() {
        return probeReady;
    }

    public void setProbeReady(boolean ready) {
        this.probeReady = ready;
    }

    /**
     * Provides two points of the scanners bounding box and the number of points to sample in the X/Y
     * directions.
     */
    public void update(final Position corner1, final Position corner2) {
        if (corner1.getUnits() != corner2.getUnits()) {
            throw new IllegalArgumentException("Provide same unit for both measures.");
        }

        double xMin = Math.min(corner1.x, corner2.x);
        double xMax = Math.max(corner1.x, corner2.x);
        double yMin = Math.min(corner1.y, corner2.y);
        double yMax = Math.max(corner1.y, corner2.y);
        double zMin = Math.min(corner1.z, corner2.z);
        double zMax = Math.max(corner1.z, corner2.z);

        Position newMin = new Position(xMin, yMin, zMin, corner1.getUnits());
        Position newMax = new Position(xMax, yMax, zMax, corner1.getUnits());

        // If we're 0 in any dimension there is nothing we can do yet.
        if (newMin.getX() != newMax.getX() && newMin.getY() != newMax.getY() && newMin.getZ() != newMax.getZ()) {
            this.minXYZ = newMin;
            this.maxXYZ = newMax;
        }

        reset();
    }

    public void handleEvent(ProbeEvent evt) {
        if (pendingPositions.isEmpty() || !isScanning.get()) {
            return;
        }

        Position probeMachinePosition = evt.getProbePosition();
        if (!Double.isFinite(probeMachinePosition.getZ())) {
            reset();
            throw new RuntimeException("Probe returned invalid position");
        }

        if (probeMachinePosition.getUnits() == Units.UNKNOWN) {
            logger.warning("Unknown units in autoleveler receiving probe. Assuming " + getPreferredUnits());
        }
        probeMachinePosition = probeMachinePosition.getPositionIn(getPreferredUnits());
        Position probePosition = probeMachinePosition.add(machineWorkOffset);

        logger.log(Level.INFO, "Record ({0}, {1}, {2})",
                new Object[]{probePosition.getX(), probePosition.getY(), probePosition.getZ()});
        probeEvent(probePosition);

        if (pendingPositions.isEmpty()) {
            // The probing is done!
            moveToSafeStartPoint(probePosition);
        } else {
            //double retractedZ = retract(probePosition.getZ());
            probeNextPoint(null);
        }
    }

    private Units getPreferredUnits() {
        return this.backend.getSettings().getPreferredUnits();
    }

    public void reset() {
        isScanning.set(false);
        int xAxisPoints = settings.getXSampleCount();
        int yAxisPoints = settings.getYSampleCount();
        this.probePositionGrid = new Position[xAxisPoints][yAxisPoints];

        // Calculate probe locations.
        double xRange = maxXYZ.getX() - minXYZ.getX();
        double yRange = maxXYZ.getY() - minXYZ.getY();

        for (int x = 0; x < xAxisPoints; x++) {
            for (int y = 0; y < yAxisPoints; y++) {
                double xStep = xAxisPoints > 1 ? (xRange * x) / (xAxisPoints - 1) : 0;
                double yStep = yAxisPoints > 1 ? (yRange * y) / (yAxisPoints - 1) : 0;

                Position p = new Position(
                        minXYZ.getX() + xStep,
                        minXYZ.getY() + yStep,
                        Double.NaN,
                        minXYZ.getUnits());
                probePositionGrid[x][y] = p;
            }
        }
        // Move along grid in zigzag pattern
        int yIncrement = 1;
        int yIndex = 0;
        pendingPositions = new LinkedList<>();
        for (Position[] columnPositions : probePositionGrid) {
            while (yIndex >= 0 && yIndex < columnPositions.length) {
                Position p = columnPositions[yIndex];
                pendingPositions.add(p);
                yIndex += yIncrement;
            }
            yIncrement = -yIncrement;
            yIndex += yIncrement;
        }

        listeners.forEach(SurfaceScannerListener::onScannerUpdate);
    }

    public void probeEvent(final Position p) {
        Position expectedProbePosition = pendingPositions.pop();
        Position probedPosition = p.getPositionIn(expectedProbePosition.getUnits());
        Position settingsOffset = settings.getAutoLevelProbeOffset().getPositionIn(getPreferredUnits());

        expectedProbePosition.setX(expectedProbePosition.getX() + settingsOffset.getX());
        expectedProbePosition.setY(expectedProbePosition.getY() + settingsOffset.getY());
        expectedProbePosition.setZ(probedPosition.getZ() + settingsOffset.getZ());
        listeners.forEach(SurfaceScannerListener::onScannerUpdate);
    }

    /**
     * Begin a scan the surface {@link #handleEvent(ProbeEvent)} must be called to properly progress through
     * the scan.
     */
    public void scan() {
        isScanning.set(true);
        Position work = backend.getWorkPosition();
        Position machine = backend.getMachinePosition();
        machineWorkOffset = new Position(work.getUnits());
        machineWorkOffset.x = work.x - machine.x;
        machineWorkOffset.y = work.y - machine.y;
        machineWorkOffset.z = work.z - machine.z;

        moveToSafeStartPoint(work);
        probeNextPoint(maxXYZ.getZ());
    }

    private void moveToSafeStartPoint(Position currentPosition) {
    try {
        // Move to the first XY point
        first = minXYZ.getPositionIn(getPreferredUnits());
        moveXYAndWait(first, 15000);

        setProbeReady(true);

    } catch (Exception e) {
        setProbeReady(false);
        reset();
        throw new RuntimeException(e);
    }
}


    public Optional<Position> getNextProbePoint() {
        return Optional.ofNullable(this.pendingPositions.peek());
    }

    private void waitForControllerIdle(long timeoutMs) throws InterruptedException {
        long start = System.currentTimeMillis();
        boolean sawNotIdle = false;

        while (System.currentTimeMillis() - start < timeoutMs) {
            boolean idle = backend.isIdle();

            if (!idle) {
                sawNotIdle = true; // we observed motion / busy state
            } else if (sawNotIdle) {
                return; // busy -> idle transition observed
            }

            Thread.sleep(25);
        }

        // If we never saw not-idle, the move may have completed extremely fast.
        // If we saw not-idle but didn't return idle, it's a real timeout.
        if (sawNotIdle) {
            throw new RuntimeException("Timeout waiting for controller to become idle.");
        }
    }
	
private void moveXYAndWait(Position target, long timeoutMs) throws Exception {
    // Use preferred units for all comparisons
    UnitUtils.Units preferred = backend.getSettings().getPreferredUnits();

    // Ensure target is in the same units we will compare against
    Position t = target.getPositionIn(preferred);

    // Build XY-only move in the same units (G90 absolute, G0 rapid)
    PartialPosition startPos = PartialPosition.builder(t)
            .clearZ()
            .clearABC()
            .build();

    String cmd = GcodeUtils.generateMoveCommand("G90G0", getProbeScanFeedRate(), startPos);
    logger.log(Level.INFO, "MoveTo {0} {1}", new Object[]{startPos, cmd});
    backend.sendGcodeCommand(true, cmd);

    // Wait for controller to be idle AND at the requested XY (in preferred units)
    long start = System.currentTimeMillis();

    // Tolerance in preferred units
    double tol = (preferred == UnitUtils.Units.MM) ? 0.05 : 0.002;

    while (System.currentTimeMillis() - start < timeoutMs) {
        // Convert current work position to preferred units
        Position w = backend.getWorkPosition().getPositionIn(preferred);

        boolean atXY =
                Math.abs(w.getX() - t.getX()) <= tol &&
                Math.abs(w.getY() - t.getY()) <= tol;

        if (backend.isIdle() && atXY) {
            return;
        }

        Thread.sleep(25);
    }

    throw new RuntimeException("Timeout waiting for XY move to complete.");
}

    private void probeNextPoint(Double zBackoff) {
    try {
        if (!isProbeReady()) {
            throw new RuntimeException("Probe not ready.");
        }

        Position target = pendingPositions.peek();
        if (target == null) return;

        // Move to the XY point and wait until motion completes
        moveXYAndWait(target, 15000);

		double zr = settings.getZRetract();
		sendZRetractToProbe(zr);


        // Now read probe measurement
        String resp = serialControl.sendAndReceiveProbe("P\n", 9000);
        if (resp == null) {
            setProbeReady(false);
            throw new RuntimeException("No response from probe to P.");
        }

        resp = resp.trim();

		if ("E".equalsIgnoreCase(resp) || "ERROR".equalsIgnoreCase(resp)) {
			setProbeReady(false);
			reset();

			// Send to output (fallback)
			System.out.println("[Probe] Over Travel Error");

			// Popup (NetBeans platform)
			try {
				org.openide.DialogDisplayer.getDefault().notify(
					new org.openide.NotifyDescriptor.Message(
						"Over Travel Error",
						org.openide.NotifyDescriptor.ERROR_MESSAGE
					)
				);
			} catch (Throwable ignore) {}

			return;
		}


        double z;
        try {
            z = Double.parseDouble(resp);
        } catch (NumberFormatException ex) {
            setProbeReady(false);
            throw new RuntimeException("Invalid probe numeric response: " + resp, ex);
        }

        if (!Double.isFinite(z)) {
            setProbeReady(false);
            throw new RuntimeException("Non-finite probe value: " + resp);
        }

        // Record result at this XY
        Position measured = new Position(target.getX(), target.getY(), z, getPreferredUnits());
        probeEvent(measured); // should pop pendingPositions

		if (!pendingPositions.isEmpty()) {
			probeNextPoint(null);
		} else {
			// Done probing all points: retract probe out of the way
			resp = serialControl.sendAndReceiveProbe("H\n", 9000);

			if (resp == null) {
				setProbeReady(false);
				throw new RuntimeException("No response from probe on final retract (H).");
			}

			resp = resp.trim();

			if ("E".equalsIgnoreCase(resp) || "ERROR".equalsIgnoreCase(resp)) {
				setProbeReady(false);
				throw new RuntimeException("Probe error on final retract (H): " + resp);
			}

			// If your probe replies "homed" on success, keep this check.
			// If it replies something else, change this accordingly.
			if (!"homed".equalsIgnoreCase(resp)) {
				throw new RuntimeException("Unexpected probe response on final retract (H): " + resp);
			}

			// Optional: return machine to first probe XY location (if you stored it)
			if (first != null) {
				moveXYAndWait(first, 15000);
			}
		}


    } catch (Exception e) {
        setProbeReady(false);
        reset();
        throw new RuntimeException(e);
    }
}

private boolean sendZRetractToProbe(double zr) {
    String cmd = String.format(java.util.Locale.US, "ZR:%.3f\n", zr);

	String resp;
	try {
		resp = serialControl.sendAndReceiveProbe(cmd,9000);   // assumes it returns a line String (or null/empty on timeout)
	} catch (Exception ex) {
		showProbePopupAndOutput("Over Travel Error"); // or "Failed to send Z retract"
		return false;
	}

	if (resp == null) {
		showProbePopupAndOutput("Over Travel Error"); // or "No response from probe"
		return false;
	}

	resp = resp.trim();
	if (resp.equalsIgnoreCase("E") || resp.equalsIgnoreCase("ERROR") || resp.toUpperCase().startsWith("ERR")) {
		showProbePopupAndOutput("Over Travel Error");
		return false;
	}

	// Accept common ACKs: "OK", "OK ZR=0.750", etc.
	if (!resp.toUpperCase().startsWith("OK")) {
		showProbePopupAndOutput("Over Travel Error"); // or "Unexpected probe response: " + resp
		return false;
	}

	return true;
}

// Put this in the same class (SurfaceScanner / AutoLevelerPanel) where you're doing the check.
private void showProbePopupAndOutput(String msg) {
    // Output window fallback (no extra dependencies)
    System.out.println("[Probe] " + msg);

    // Popup (NetBeans Platform)
    try {
        org.openide.DialogDisplayer.getDefault().notify(
            new org.openide.NotifyDescriptor.Message(
                msg,
                org.openide.NotifyDescriptor.ERROR_MESSAGE
            )
        );
    } catch (Throwable ignore) {}
}

	private double getProbeScanFeedRate() {
        return settings.getProbeScanFeedRate() * UnitUtils.scaleUnits(Units.MM, getPreferredUnits());
    }

    private double retract(Double zLast) {
        double zRetract = settings.getZRetract() * maxXYZ.getZ();
        if (zRetract <= 0) {
            zRetract = maxXYZ.getZ() - minXYZ.getZ();
        }

        // Start by backing off the current position
        double zBackoff = Math.min(zLast + zRetract, maxXYZ.getZ());
        PartialPosition safeZ = PartialPosition.builder(maxXYZ.getUnits()).setZ(zBackoff).build();
        String retractCommand = GcodeUtils.generateMoveCommand(
                "G90G0",
                getProbeScanFeedRate(),
                safeZ);

        try {
            logger.log(Level.INFO, "Retract to {0} {1}", new Object[]{safeZ, retractCommand});
            backend.sendGcodeCommand(true, retractCommand);
        } catch (Exception e) {
            reset();
            throw new RuntimeException(e);
        }
        return zBackoff;
    }

    public void scanRandomData() {
        machineWorkOffset.x = 0;
        machineWorkOffset.y = 0;
        machineWorkOffset.z = 0;

        // Generate some random test data.
        while (!pendingPositions.isEmpty()) {
            Position p = new Position(pendingPositions.peek());
            p.setZ(ThreadLocalRandom.current().nextDouble(minXYZ.getZ(), maxXYZ.getZ()));
            probeEvent(p);
        }

        listeners.forEach(SurfaceScannerListener::onScannerUpdate);
    }

    public ImmutableList<Position> getProbeStartPositions() {
        ImmutableList.Builder<Position> builder = ImmutableList.builder();
        double z = maxXYZ.getZ();
        for (Position[] columns : probePositionGrid) {
            for (Position p : columns) {
                Position zMaxPoint = new Position(p);
                zMaxPoint.setZ(z);
                builder.add(zMaxPoint);
            }
        }
        return builder.build();
    }

    public final Position[][] getProbePositionGrid() {
        return this.probePositionGrid;
    }

    public boolean isValid() {
        return probePositionGrid.length > 0 && pendingPositions.isEmpty();
    }

    public void addListener(SurfaceScannerListener listener) {
        listeners.add(listener);
    }
}
