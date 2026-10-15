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
    private final AtomicBoolean stopRequested = new AtomicBoolean(false);
    private volatile Thread scanThread = null;
    // Registered with the pendant only while this scanner is scanning
    private final java.util.function.BooleanSupplier pendantStopHandler = this::requestStop;

    /**
     * Thrown out of {@link #scan()} when the user stopped the scan.
     */
    public static class ScanStoppedException extends RuntimeException {
        public ScanStoppedException() {
            this("Scan stopped. Home the probe before scanning again.");
        }

        public ScanStoppedException(String message) {
            super(message);
        }
    }

    /**
     * Thrown out of {@link #scan()} when the probe over travelled. The probe has already been
     * told to retract; the message says whether it confirmed.
     */
    public static class ProbeOverTravelException extends RuntimeException {
        public ProbeOverTravelException(String message) {
            super(message);
        }
    }

    public SurfaceScanner(BackendAPI backend) {
        this.backend = backend;
        this.settings = backend.getSettings().getAutoLevelSettings();
        Position minPosition = new Position(settings.getMinX(), settings.getMinY(), settings.getMinZ(), backend.getSettings().getPreferredUnits());
        Position maxPosition = new Position(settings.getMaxX(), settings.getMaxY(), settings.getMaxZ(), backend.getSettings().getPreferredUnits());
        update(minPosition, maxPosition);
    }

    public boolean isScanning() {
        return scanThread != null;
    }

    /**
     * Stops a running scan: breaks the scan loop, feed-holds/resets the CNC and tells the probe to abort.
     * Safe to call from any thread.
     *
     * @return true if a scan was running and is being stopped
     */
    public boolean requestStop() {
        Thread t = scanThread;
        if (t == null) {
            return false;
        }

        logger.log(Level.INFO, "Stop requested for surface scan");
        stopRequested.set(true);

        // Wake the scan thread out of any sleep / serial wait. Done before the probe abort so a probe
        // command not yet written is dropped instead of reaching the probe after the abort.
        t.interrupt();

        // Stop the probe's own Z motion
        serialControl.abortProbe();

        // Stop the CNC: feed hold, then GRBL soft-resets once motion has stopped (keeps position)
        try {
            backend.cancel();
        } catch (Exception e) {
            logger.log(Level.WARNING, "Could not cancel controller motion", e);
        }
        return true;
    }

    private void checkStop() {
        if (stopRequested.get()) {
            throw new ScanStoppedException();
        }
    }

    /**
     * backend.cancel() in {@link #requestStop()} does nothing while UGS still thinks the machine is idle,
     * which is the case for up to one status poll after a move was sent. Watch briefly and cancel any
     * move that shows up. Runs on the scan thread.
     */
    private void haltMachineAfterStop() {
        Thread.interrupted(); // clear the stop interrupt so we can wait
        long end = System.currentTimeMillis() + 1000;
        try {
            while (System.currentTimeMillis() < end) {
                if (backend.canPause()) {
                    backend.cancel();
                    return;
                }
                Thread.sleep(50);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            logger.log(Level.WARNING, "Could not cancel controller motion after stop", e);
        }
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
            probeRemainingPoints();
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
        stopRequested.set(false);
        scanThread = Thread.currentThread();
        isScanning.set(true);
        // Let the pendant's cancel key stop this scan
        serialControl.setScanStopHandler(pendantStopHandler);
        listeners.forEach(SurfaceScannerListener::onScannerUpdate);

        try {
            Position work = backend.getWorkPosition();
            Position machine = backend.getMachinePosition();
            machineWorkOffset = new Position(work.getUnits());
            machineWorkOffset.x = work.x - machine.x;
            machineWorkOffset.y = work.y - machine.y;
            machineWorkOffset.z = work.z - machine.z;

            moveToSafeStartPoint(work);
            probeRemainingPoints();
        } finally {
            serialControl.clearScanStopHandler(pendantStopHandler);
            scanThread = null;
            isScanning.set(false);
            // Don't leave an interrupt from requestStop() on the pooled worker thread
            Thread.interrupted();
            listeners.forEach(SurfaceScannerListener::onScannerUpdate);
        }
    }

    /**
     * Resets the scan and returns the exception to throw: the {@link ProbeOverTravelException} as is,
     * a {@link ScanStoppedException} if the user stopped the scan, otherwise the original failure.
     */
    private RuntimeException scanFailed(Exception e) {
        setProbeReady(false);
        reset();
        if (e instanceof ProbeOverTravelException overTravel) {
            return overTravel;
        }
        if (stopRequested.get() || e instanceof ScanStoppedException) {
            haltMachineAfterStop();
            return new ScanStoppedException();
        }
        return new RuntimeException(e);
    }

    private void moveToSafeStartPoint(Position currentPosition) {
    try {
        checkStop();

        // Move to the first XY point
        first = minXYZ.getPositionIn(getPreferredUnits());
        moveXYAndWait(first, 15000);

        setProbeReady(true);

    } catch (Exception e) {
        throw scanFailed(e);
    }
}

    /**
     * Probes every pending point in order, then retracts the probe. Checks for a stop request between points.
     */
    private void probeRemainingPoints() {
        while (!pendingPositions.isEmpty()) {
            probeNextPoint();
        }
        finishScan();
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
	
private void moveXYAndWait(Position target, long minTimeoutMs) throws Exception {
    UnitUtils.Units preferred = backend.getSettings().getPreferredUnits();
    Position t = target.getPositionIn(preferred);

    PartialPosition startPos = PartialPosition.builder(t)
            .clearZ()
            .clearABC()
            .build();

    String cmd = GcodeUtils.generateMoveCommand("G90G0", getProbeScanFeedRate(), startPos);
    logger.log(Level.INFO, "MoveTo {0} {1}", new Object[]{startPos, cmd});

    // Calculate expected travel time with a 3x safety margin
    Position currentWork = backend.getWorkPosition().getPositionIn(preferred);
    double dx = t.getX() - currentWork.getX();
    double dy = t.getY() - currentWork.getY();
    double distance = Math.sqrt(dx * dx + dy * dy);

    // Feed rate is in units-per-minute; convert to units-per-millisecond
    double feedRate = getProbeScanFeedRate();  // units/min
    // No usable feed rate: fall back to the minimum timeout instead of waiting forever
    double expectedMs = feedRate > 0 ? (distance / feedRate) * 60000.0 : 0;

    // Use 3x expected time, plus 2 second buffer, with a floor of minTimeoutMs
    long calculatedTimeout = (long)(expectedMs * 3.0 + 2000);
    long timeoutMs = Math.max(minTimeoutMs, calculatedTimeout);

    logger.log(Level.INFO, "Move distance={0}, expectedMs={1}, timeout={2}",
            new Object[]{distance, expectedMs, timeoutMs});

    checkStop();
    backend.sendGcodeCommand(true, cmd);

    // Give GRBL a moment to actually start moving
    Thread.sleep(150);

    long start = System.currentTimeMillis();
    double tol = (preferred == UnitUtils.Units.MM) ? 0.05 : 0.002;
    boolean sawNotIdle = false;

    while (System.currentTimeMillis() - start < timeoutMs) {
        checkStop();
        Position w = backend.getWorkPosition().getPositionIn(preferred);
        boolean idle = backend.isIdle();

        if (!idle) {
            sawNotIdle = true;
        }

        boolean atXY =
                Math.abs(w.getX() - t.getX()) <= tol &&
                Math.abs(w.getY() - t.getY()) <= tol;

        // Done if idle, at target, AND we've seen actual motion (or this was a no-op)
        if (idle && atXY && (sawNotIdle || (System.currentTimeMillis() - start > 500))) {
            return;
        }

        Thread.sleep(25);
    }

    throw new RuntimeException("Timeout waiting for XY move to complete after " + timeoutMs + "ms.");
}

    /**
     * Moves to the next pending point and probes it. An over travel retracts the probe and
     * throws {@link ProbeOverTravelException}.
     */
    private void probeNextPoint() {
    try {
        checkStop();
        if (!isProbeReady()) {
            throw new RuntimeException("Probe not ready.");
        }

        Position target = pendingPositions.peek();
        if (target == null) return;

        // Move to the XY point and wait until motion completes
        moveXYAndWait(target, 15000);

		checkStop();
		double zr = settings.getZRetract();
		sendZRetractToProbe(zr);


        // Now read probe measurement
        checkStop();
        String resp = serialControl.sendAndReceiveProbe("P\n", 9000);
        checkStop();
        if (resp == null) {
            setProbeReady(false);
            throw new RuntimeException("No response from probe to P.");
        }

        resp = resp.trim();

		if ("E".equalsIgnoreCase(resp) || "ERROR".equalsIgnoreCase(resp)) {
			throw overTravel("Probe reported over travel while probing.");
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

    } catch (Exception e) {
        throw scanFailed(e);
    }
}

    /**
     * Done probing all points: retract probe out of the way and return to the first point.
     */
    private void finishScan() {
    try {
			checkStop();
			String resp = serialControl.sendAndReceiveProbe("H\n", 9000);
			checkStop();

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

    } catch (Exception e) {
        // Every point is already probed: keep the data, only the retract / return didn't finish
        setProbeReady(false);
        if (stopRequested.get() || e instanceof ScanStoppedException) {
            haltMachineAfterStop();
            throw new ScanStoppedException("Scan complete, data kept.\n"
                    + "The final probe retract / return move was stopped. Home the probe before moving the machine.");
        }
        throw new RuntimeException("the final probe retract / return did not finish (scan data was kept): " + e.getMessage(), e);
    }
}

/**
 * Sends the Z retract distance to the probe. Any failure is treated as an over travel:
 * the probe is retracted and the scan is cancelled.
 */
private void sendZRetractToProbe(double zr) {
    String cmd = String.format(java.util.Locale.US, "ZR:%.3f\n", zr);

	String resp = serialControl.sendAndReceiveProbe(cmd, 9000);
	checkStop();

	if (resp == null) {
		throw overTravel("No response from probe to Z retract (ZR).");
	}

	resp = resp.trim();
	if (resp.equalsIgnoreCase("E") || resp.equalsIgnoreCase("ERROR") || resp.toUpperCase().startsWith("ERR")) {
		throw overTravel("Probe reported an error on Z retract (ZR).");
	}

	// Accept common ACKs: "OK", "OK ZR=0.750", etc.
	if (!resp.toUpperCase().startsWith("OK")) {
		throw overTravel("Unexpected probe response to Z retract (ZR): " + resp);
	}
}

/**
 * Retracts (homes) the probe after an over travel and returns the exception that cancels the scan.
 */
private ProbeOverTravelException overTravel(String reason) {
    logger.log(Level.WARNING, "Probe over travel: {0}", reason);
    setProbeReady(false);

    // Get the probe out of the way before giving up
    String resp = serialControl.sendAndReceiveProbe("H\n", 9000);
    if (resp != null && "homed".equalsIgnoreCase(resp.trim())) {
        return new ProbeOverTravelException("Over Travel Error\n" + reason + "\nProbe retracted, scan cancelled.");
    }

    return new ProbeOverTravelException("Over Travel Error\n" + reason
            + "\nProbe did NOT confirm retract (response: " + resp + "). Check the probe before moving the machine."
            + "\nScan cancelled.");
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
