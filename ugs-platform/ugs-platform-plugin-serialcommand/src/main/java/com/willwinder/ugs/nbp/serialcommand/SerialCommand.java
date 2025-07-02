// CNC Controller Plugin
package com.willwinder.ugs.nbp.serialcommand;

import com.fazecast.jSerialComm.SerialPort;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import com.willwinder.universalgcodesender.services.JogService;
import com.willwinder.ugs.nbp.lib.lookup.CentralLookup;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Scanner;
import org.openide.modules.OnStart;
import org.openide.util.lookup.ServiceProvider;
import org.openide.windows.IOProvider;
import org.openide.windows.InputOutput;

@OnStart
@ServiceProvider(service = Runnable.class)
public class SerialCommand implements Runnable {

    private static final int BAUD_RATE = 9600;
    private static final long CONNECTION_TIMEOUT_MS = 10000;

    private SerialPort activePort;
    private OutputStream serialOut;
    private InputStream inputStream;
    private boolean connected = false;
    private long lastNanoPing = 0;
    private final StringBuilder serialLine = new StringBuilder();
    private BackendAPI backend;
    private com.willwinder.universalgcodesender.model.BackendAPIReadOnly apiReadOnly;
    private volatile boolean collecting = false;
    private volatile boolean nanoRequestActive = false;
    private JogService jog;
    private final ByteBuffer buffer = ByteBuffer.allocate(38);

    @Override
    public void run() {
        new Thread(() -> {
            InputOutput io = IOProvider.getDefault().getIO("SerialCommand", true);
            io.select();
            backend = CentralLookup.getDefault().lookup(BackendAPI.class);
            jog = CentralLookup.getDefault().lookup(JogService.class);
            apiReadOnly = CentralLookup.getDefault().lookup(com.willwinder.universalgcodesender.model.BackendAPIReadOnly.class);
            if (apiReadOnly != null) {
                apiReadOnly.addUGSEventListener(e -> {
                    if (collecting || nanoRequestActive) return;
                    collecting = true;
                    try {
                        collectStatus();
                    } finally {
                        collecting = false;
                    }
                });
            }
            while (true) {
                if (!connected) {
                    startSerialConnection(io);
                } else {
                    heartbeatTick(io);
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException ignored) {}
            }
        }).start();
    }

    private void startSerialConnection(InputOutput io) {
        try {
            SerialPort selected = null;
            for (SerialPort port : SerialPort.getCommPorts()) {
                io.getOut().println("[Scan] Trying port: " + port.getSystemPortName());
                port.setBaudRate(BAUD_RATE);
                port.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0);
                if (!port.openPort()) {
                    io.getOut().println("[Scan] Failed to open: " + port.getSystemPortName());
                    continue;
                }

                Scanner tempScanner = new Scanner(port.getInputStream());
                Thread.sleep(100);
                boolean found = false;

                while (tempScanner.hasNextLine()) {
                    String line = tempScanner.nextLine().trim();
                    io.getOut().println("[Scan] Read: " + line);
                    if ("CONREQ".equals(line)) {
                        serialOut = port.getOutputStream();
                        serialOut.write("CONACK\n".getBytes());
                        serialOut.flush();
                        Thread.sleep(100); // allow Nano time to process
                        selected = port;
                        io.getOut().println("[Handshake] CONACK sent on " + port.getSystemPortName());
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    port.closePort();
                } else {
                    break;
                }
            }

            if (selected == null) {
                io.getErr().println("[Error] No compatible port found.");
                return;
            }

            activePort = selected;
            serialOut = activePort.getOutputStream();
            inputStream = activePort.getInputStream();

            connected = true;
            lastNanoPing = System.currentTimeMillis();
            io.getOut().println("[Connected] Active port: " + activePort.getSystemPortName());

        } catch (Exception ex) {
            io.getErr().println("[Exception] " + ex.getMessage());
        }
    }

    public void heartbeatTick(InputOutput io) {
        if (!connected) {
            return;
        }

        try {
            while (inputStream.available() > 0) {
                int b = inputStream.read();
                if (b == -1) break;
                char c = (char) b;
                if (c == '\r') continue;
                serialLine.append(c);
                if (c == '\n') {
                    String line = serialLine.toString().trim();
                    serialLine.setLength(0);
                    io.getOut().println("[Recv] " + line);

                    if ("NANO".equals(line)) {
                        lastNanoPing = System.currentTimeMillis();
                        serialOut.write("UGS\n".getBytes());
                        serialOut.flush();
                        Thread.sleep(100);
                    } else if (line.startsWith("KEY:")) {
                        int code = Integer.parseInt(line.substring(4));
                        handleCode(code);
                    } else if ("REQ".equals(line)) {
                        nanoRequestActive = true;
                        pushBuffer();
                        nanoRequestActive = false;
                    }
                }
            }

            if (System.currentTimeMillis() - lastNanoPing > CONNECTION_TIMEOUT_MS) {
                connected = false;
                io.getErr().println("[Timeout] Lost connection to nano.");
                if (activePort != null) {
                    activePort.closePort();
                }
            }
        } catch (Exception e) {
            connected = false;
            io.getErr().println("[Error] Heartbeat exception: " + e.getMessage());
        }
    }

    private void collectStatus() {
        if (backend == null || jog == null) return;

        Position p = backend.getWorkPosition();
        boolean metric = jog.getUnits() == Units.MM;

        float x = (float) p.getX();
        float y = (float) p.getY();
        float z = (float) p.getZ();
        float a = (float) (Double.isNaN(p.getA()) ? 0 : p.getA());

        if (!metric) {
            x /= 25.4;
            y /= 25.4;
            z /= 25.4;
            a /= 25.4;
        }

        int feedRate = jog.getFeedRate();
        float stepSize = (float) jog.getStepSizeXY();

        buffer.clear();
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.putFloat(x);
        buffer.putFloat(y);
        buffer.putFloat(z);
        buffer.putFloat(a);
        buffer.putFloat(feedRate);
        buffer.putFloat(stepSize);
    }

    private void pushBuffer() {
        if (!connected || serialOut == null) return;

        try {
            serialOut.write("SYNC\n".getBytes());
            serialOut.write(buffer.array());
            serialOut.flush();
        } catch (Exception ignored) {}
    }

    private void handleCode(int code) {
        if (backend == null || jog == null) return;

        try {
            switch (code) {
                case 0: jog.adjustManualLocationZ(-1); break;
                case 1: jog.adjustManualLocationABC(-1, 0, 0); break;
                case 2: jog.adjustManualLocationABC(1, 0, 0); break;
                case 3: jog.adjustManualLocationZ(1); break;
                case 4: jog.adjustManualLocationXY(0, -1); break;
                case 5: jog.adjustManualLocationXY(1, 0); break;
                case 6: jog.adjustManualLocationXY(0, 1); break;
                case 7: jog.adjustManualLocationXY(-1, 0); break;
                case 8: setStep(jog, 0.001); break;
                case 9: setStep(jog, 0.010); break;
                case 10: setStep(jog, 0.100); break;
                case 11: setStep(jog, 1.000); break;
                case 12: backend.resetCoordinateToZero(Axis.A); break;
                case 13: backend.resetCoordinateToZero(Axis.Z); break;
                case 14: backend.resetCoordinateToZero(Axis.Y); break;
                case 15: backend.resetCoordinateToZero(Axis.X); break;
                case 16: backend.sendGcodeCommand("G53 G0 A0"); break;
                case 17: backend.sendGcodeCommand("G53 G0 Z0"); break;
                case 18: backend.sendGcodeCommand("G53 G0 Y0"); break;
                case 19: backend.sendGcodeCommand("G53 G0 X0"); break;
                case 22: backend.returnToZero(); break;
                case 23: backend.performHomingCycle(); break;
                case 25: backend.send(); break;
                case 26: backend.pauseResume(); break;
                case 27: backend.cancel(); break;
            }
        } catch (Exception ignored) {}
    }

    private static void setStep(JogService j, double v) {
        j.setStepSizeZ(v);
        j.setStepSizeXY(v);
        j.setStepSizeABC(v);
    }
}
