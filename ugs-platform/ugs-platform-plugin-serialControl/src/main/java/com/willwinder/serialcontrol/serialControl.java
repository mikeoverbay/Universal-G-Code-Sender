// CNC Controller Plugin
package com.willwinder.serialcontrol;

import com.fazecast.jSerialComm.SerialPort;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.BackendAPIReadOnly;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.services.JogService;
import com.willwinder.ugs.nbp.lib.lookup.CentralLookup;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import org.openide.modules.OnStart;
import org.openide.util.lookup.ServiceProvider;
import org.openide.windows.IOProvider;
import org.openide.windows.InputOutput;
import java.nio.ByteOrder;
import com.willwinder.universalgcodesender.model.events.ControllerStatusEvent;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;
import com.willwinder.universalgcodesender.listeners.ControllerStatus;



@OnStart
@ServiceProvider(service = Runnable.class)
public class serialControl implements Runnable {

    private static final int BAUD_RATE = 9600;
    private static final long CONNECTION_TIMEOUT_MS = 10000;

    private SerialPort activePort;
    private OutputStream serialOut;
    private InputStream inputStream;
    private boolean connected = false;
    private long lastNanoPing = 0;
    private JogService jogService;
    private BackendAPI backend;
    private BackendAPIReadOnly apiReadOnly;
    private static InputOutput io;

    private volatile int liveFeedRate = 0;
    private volatile int liveSpindleSpeed = 0;

    /* -------------------------------------------------- */
    /*  Helper                                             */
    /* -------------------------------------------------- */
    private static void initIO() {
        if (io == null) {
            io = IOProvider.getDefault().getIO("serialcontrol", false);
            io.select(); // show tab once
        }
    }

    @Override
    public void run() {
         // Ensure IO is ready **before** starting serial thread
        initIO();
        new Thread(this::startSerialConnection, "SerialControl-ConnectionThread").start();
        io = IOProvider.getDefault().getIO("serialcontrol", false);
        io.select(); // Only selects once
        backend = CentralLookup.getDefault().lookup(BackendAPI.class);
        jogService = CentralLookup.getDefault().lookup(JogService.class);
        apiReadOnly = CentralLookup.getDefault().lookup(BackendAPIReadOnly.class);

        if (apiReadOnly != null) {
            apiReadOnly.addUGSEventListener(e -> {
                if (e instanceof ControllerStatusEvent controllerStatusEvent) {

                    ControllerStatus status = controllerStatusEvent.getStatus();

                    // Use real-time values if available, otherwise show the target values.
                    liveFeedRate = status.getFeedSpeed() != null
                            ? (int) (status.getFeedSpeed() * UnitUtils.scaleUnits(status.getFeedSpeedUnits(), backend.getSettings().getPreferredUnits()))
                            : (int) this.backend.getGcodeState().feedRate;

                    liveSpindleSpeed = status.getSpindleSpeed() != null
                            ? status.getSpindleSpeed().intValue()
                            : (int) this.backend.getGcodeState().spindleSpeed;

                    //io.getOut().println("[Live] Feed: " + liveFeedRate + " RPM: " + liveSpindleSpeed);
                }
            });
        }
    }

    private void startSerialConnection() {

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

                try (java.util.Scanner tempScanner = new java.util.Scanner(port.getInputStream())) {
                    Thread.sleep(2000);
                    boolean found = false;
                    while (tempScanner.hasNextLine()) {
                        String line = tempScanner.nextLine().trim();
                        io.getOut().println("[Scan] Read: " + line);
                        if ("CONREQ".equals(line)) {
                            serialOut = port.getOutputStream();
                            serialOut.write("CONACK\n".getBytes());
                            serialOut.flush();
                            Thread.sleep(100); // allow stream and Nano time to process
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

            StringBuilder serialLine = new StringBuilder();

            while (connected) {
                while (inputStream.available() > 0) {
                    int b = inputStream.read();
                    if (b == -1) {
                        break;
                    }
                    char c = (char) b;
                    if (c == '\r') {
                        continue;
                    }
                    serialLine.append(c);
                    if (c == '\n') {
                        String line = serialLine.toString().trim();
                        serialLine.setLength(0);
//                        io.getOut().println("[Recv] " + line);

                        if (line.startsWith("KEY:")) {
                            int code = Integer.parseInt(line.substring(4));
                            handleCode(code, io);
                            sendSnapshot();
                            Thread.sleep(30);
                        } else if ("REQ".equals(line)) {
                            lastNanoPing = System.currentTimeMillis();
                            sendSnapshot();
                            Thread.sleep(30);
                        }
                    }
                }

                if (System.currentTimeMillis() - lastNanoPing > CONNECTION_TIMEOUT_MS) {
                    io.getErr().println("[Timeout] Lost connection to nano.");
                    connected = false;
                    activePort.closePort();
                    break;
                }

                Thread.sleep(10);
            }

        } catch (Exception ex) {
            io.getErr().println("[Exception] " + ex.getMessage());
            connected = false;
        }
    }

    private void writeFloatToBuffer(ByteBuffer buf, float value) {
        if (Float.isNaN(value)) {
            value = 0;
        }
        buf.order(ByteOrder.LITTLE_ENDIAN);
        buf.putFloat(value);
    }

    private void sendSnapshot() {

        try {
            if (activePort.bytesAwaitingWrite() > 0) {
                // Previous data still pending, skip this send to avoid flooding
                return;
            }

            Position p = apiReadOnly.getWorkPosition();
            boolean metric = (jogService.getUnits() == Units.MM);

            float x = (float) p.getX();
            float y = (float) p.getY();
            float z = (float) p.getZ();

            float a = (float) (Double.isNaN(p.getA()) ? 0 : p.getA());
            if (metric) {
                x /= 25.4;
                y /= 25.4;
                z /= 25.4;
                a /= 25.4;
            }

            long total = apiReadOnly.getNumRows();
            long sent = apiReadOnly.getNumSentRows();

            int feedRate = jogService.getFeedRate();
            float stepSize = (float) jogService.getStepSizeXY();

            float status;
            if (apiReadOnly.isPaused()) {
                status = 1;  // paused/hold
            } else if (apiReadOnly.isSendingFile()) {
                status = 2;  // running
            } else if (apiReadOnly.isIdle()) {
                status = 0;  // idle
            } else {
                status = 3;  // busy or other state

            }

            ByteBuffer packet = ByteBuffer.allocate(44);// 4 bytes for SYNC + 36 bytes(4*10)
            packet.put((byte) 'S');
            packet.put((byte) 'Y');
            packet.put((byte) 'N');
            packet.put((byte) 'C');

            writeFloatToBuffer(packet, x);
            writeFloatToBuffer(packet, y);
            writeFloatToBuffer(packet, z);
            writeFloatToBuffer(packet, a);
            writeFloatToBuffer(packet, status);
            writeFloatToBuffer(packet, feedRate);
            writeFloatToBuffer(packet, sent);
            writeFloatToBuffer(packet, total);
            writeFloatToBuffer(packet, stepSize);
            writeFloatToBuffer(packet, liveFeedRate);

            serialOut.write(packet.array());
            serialOut.flush();
            //io.getOut().println("[Sent Snapshot] FeedRate=" + liveFeedRate + " SpindleSpeed=" + liveSpindleSpeed);

        } catch (Exception e) {

        }
    }

    private void handleCode(int code, InputOutput io) {
        if (backend == null || jogService == null) {
            io.getErr().println("[Error] Backend or JogService unavailable.");
            return;
        }

        try {
            switch (code) {
                case 0:
                    jogService.adjustManualLocationZ(-1);
                    break;
                case 1:
                    jogService.adjustManualLocationABC(-1, 0, 0);
                    break;
                case 2:
                    jogService.adjustManualLocationABC(1, 0, 0);
                    break;
                case 3:
                    jogService.adjustManualLocationZ(1);
                    break;
                case 4:
                    jogService.adjustManualLocationXY(0, -1);
                    break;
                case 5:
                    jogService.adjustManualLocationXY(1, 0);
                    break;
                case 6:
                    jogService.adjustManualLocationXY(0, 1);
                    break;
                case 7:
                    jogService.adjustManualLocationXY(-1, 0);
                    break;
                case 8:
                    setStep(jogService, 0.001);
                    break;
                case 9:
                    setStep(jogService, 0.010);
                    break;
                case 10:
                    setStep(jogService, 0.100);
                    break;
                case 11:
                    setStep(jogService, 1.000);
                    break;
                case 12:
                    backend.resetCoordinateToZero(Axis.A);
                    break;
                case 13:
                    backend.resetCoordinateToZero(Axis.Z);
                    break;
                case 14:
                    backend.resetCoordinateToZero(Axis.Y);
                    break;
                case 15:
                    backend.resetCoordinateToZero(Axis.X);
                    break;
                case 16:
                    backend.sendGcodeCommand("G53 G0 A0");
                    break;
                case 17:
                    backend.sendGcodeCommand("G53 G0 Z0");
                    break;
                case 18:
                    backend.sendGcodeCommand("G53 G0 Y0");
                    break;
                case 19:
                    backend.sendGcodeCommand("G53 G0 X0");
                    break;
                case 22:
                    backend.returnToZero();
                    break;
                case 23:
                    backend.performHomingCycle();
                    break;
                case 25:
                    backend.send();
                    break;
                case 26:
                    backend.pauseResume();
                    break;
                case 27:
                    backend.cancel();
                    break;
                default:
                    io.getErr().println("[Warning] Unknown code: " + code);
            }
        } catch (Exception ex) {
            io.getErr().println("[Error] Handling code " + code + ": " + ex.getMessage());
        }
    }

    private static void setStep(JogService j, double v) {
        j.setStepSizeZ(v);
        j.setStepSizeXY(v);
        j.setStepSizeABC(v);
    }
}
