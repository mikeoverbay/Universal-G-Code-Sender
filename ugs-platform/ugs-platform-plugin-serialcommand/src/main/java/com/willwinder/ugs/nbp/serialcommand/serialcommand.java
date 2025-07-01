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
package com.willwinder.ugs.nbp.serialcommand;

/**
 *
 * @author Mike Overbay
 */
import com.fazecast.jSerialComm.SerialPort;
import com.willwinder.universalgcodesender.model.Axis;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.BackendAPIReadOnly;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils.Units;

import com.willwinder.universalgcodesender.services.JogService;
import com.willwinder.ugs.nbp.lib.lookup.CentralLookup;
import org.openide.modules.OnStart;
import org.openide.util.lookup.ServiceProvider;
import org.openide.windows.IOProvider;
import org.openide.windows.InputOutput;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Scanner;

@OnStart
@ServiceProvider(service = Runnable.class)
public class SerialCommand implements Runnable {

    private SerialPort activePort;
    private OutputStream serialOut;
    private boolean connected = false;
    private JogService jogService;

    private BackendAPIReadOnly apiReadOnly;

    public SerialCommand() {
        this.jogService = null;
    }

    @Override
    public void run() {
        jogService = CentralLookup.getDefault().lookup(JogService.class);
        new Thread(this::startSerialConnection).start();
    }

    private void startSerialConnection() {
        InputOutput io = IOProvider.getDefault().getIO("SerialCommand", true);
        io.select();

        try {
            SerialPort selected = null;
            for (SerialPort port : SerialPort.getCommPorts()) {
                io.getOut().println("[SerialCommand] Trying port: " + port.getSystemPortName());
                port.setComPortTimeouts(SerialPort.TIMEOUT_SCANNER, 1000, 0);
                if (!port.openPort()) {
                    io.getOut().println("[SerialCommand] Failed to open port: " + port.getSystemPortName());
                    continue;
                }

                try (Scanner testScanner = new Scanner(port.getInputStream())) {
                    if (testScanner.hasNextLine()) {
                        String line = testScanner.nextLine().trim();
                        io.getOut().println("[SerialCommand] Read line: " + line);
                        if ("KEY:-1".equals(line)) {
                            selected = port;
                            io.getOut().println("[SerialCommand] Pendant on " + port.getSystemPortName());
                            break;
                        }
                    }
                } catch (Exception ignore) {
                } finally {
                    if (selected == null) {
                        port.closePort();
                    }
                }
            }

            if (selected == null) {
                io.getErr().println("[SerialCommand] Pendant not found.");
                return;
            }

            selected.setBaudRate(115200);
            InputStream in = selected.getInputStream();
            serialOut = selected.getOutputStream();
            activePort = selected;
            Scanner scanner = new Scanner(in);
            connected = true;

            apiReadOnly = CentralLookup.getDefault().lookup(BackendAPIReadOnly.class);
            if (apiReadOnly != null) {
                apiReadOnly.addUGSEventListener(e -> pushUpdate());
                pushUpdate();
            } else {
                io.getErr().println("[SerialCommand] BackendAPIReadOnly unavailable.");
            }

            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (!"KEY:-1".equals(line)) {
                    io.getOut().println("[Pendant] " + line);
                }
                if (line.startsWith("KEY:")) {
                    try {
                        int code = Integer.parseInt(line.substring(4));
                        handleCode(code, io);
                    } catch (NumberFormatException ex) {
                        io.getErr().println("[SerialCommand] Bad key: " + line);
                    }
                }
            }
        } catch (Exception ex) {
            io.getErr().println("[SerialCommand] Serial error: " + ex.getMessage());
        }
    }

    private void sendToPendant(String msg) {
        try {
            if (serialOut != null && activePort != null && activePort.bytesAwaitingWrite() == 0) {
                serialOut.write((msg + "\n").getBytes());
                serialOut.flush();
            }
        } catch (IOException ex) {
            System.err.println("[SerialCommand] TX fail: " + ex.getMessage());
        }
    }

    private static String cell(String s) {
        return String.format("%-10.10s", s);
    }

    private static String axis(String l, double v) {
        return String.format("|%s%+06.3f", l, v);
    }

    private void pushUpdate() {
        if (!connected || apiReadOnly == null) {
            return;
        }
        try {
            if (activePort != null && activePort.bytesAwaitingWrite() > 0) {
                return;
            }

            Position p = apiReadOnly.getWorkPosition();
            
            boolean metric = (jogService.getUnits() == Units.MM);

            double x = p.getX(), y = p.getY(), z = p.getZ(), a = p.getA();
            if (!metric) {
                x /= 25.4;
                y /= 25.4;
                z /= 25.4;
                a /= 25.4;
				a = Double.isNaN(a) ? 0 : a / 25.4;
            }

            long total = apiReadOnly.getNumRows();
            long sent = apiReadOnly.getNumSentRows();

            int feedRate = 0;
            JogService js = CentralLookup.getDefault().lookup(JogService.class);
            if (js != null) {
                feedRate = js.getFeedRate();
            }

            String status = apiReadOnly.isPaused() ? "PAUSE"
                    : apiReadOnly.isSendingFile() ? "RUN"
                    : apiReadOnly.isIdle() ? "IDLE" : "HOLD";

            String row0 = cell(status) + cell(axis("X", x));
            String row1 = cell("F" + feedRate) + cell(axis("Y", y));
            String row2 = cell("") + cell(axis("Z", z));
            String row3 = cell(String.format("L%d/%d", sent, total)) + cell(axis("A", a));

            String lcdBlock = "LCD:0:" + row0 + "\r"
                    + "LCD:1:" + row1 + "\r"
                    + "LCD:2:" + row2 + "\r"
                    + "LCD:3:" + row3 + "\r";
            sendToPendant(lcdBlock);
        } catch (Exception ignore) {
        }
    }

    private void handleCode(int code, InputOutput io) {
        BackendAPI backend = CentralLookup.getDefault().lookup(BackendAPI.class);
        JogService jog = CentralLookup.getDefault().lookup(JogService.class);
        if (backend == null || jog == null) {
            io.getErr().println("[SerialCommand] UGS services unavailable.");
            return;
        }
        try {
            switch (code) {
                case 0:
                    jog.adjustManualLocationZ(-1);
                    break;
                case 1:
                    jog.adjustManualLocationABC(-1, 0, 0);
                    break;
                case 2:
                    jog.adjustManualLocationABC(1, 0, 0);
                    break;
                case 3:
                    jog.adjustManualLocationZ(1);
                    break;
                case 4:
                    jog.adjustManualLocationXY(0, -1);
                    break;
                case 5:
                    jog.adjustManualLocationXY(1, 0);
                    break;
                case 6:
                    jog.adjustManualLocationXY(0, 1);
                    break;
                case 7:
                    jog.adjustManualLocationXY(-1, 0);
                    break;
                case 8:
                    setStep(jog, 0.001);
                    break;
                case 9:
                    setStep(jog, 0.010);
                    break;
                case 10:
                    setStep(jog, 0.100);
                    break;
                case 11:
                    setStep(jog, 1.000);
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
                    break;
            }
           
        } catch (Exception ex) {
            io.getErr().println("[SerialCommand] Error handling code " + code + ": " + ex.getMessage());
        }
    }

    private static void setStep(JogService j, double v) {
        j.setStepSizeZ(v);
        j.setStepSizeXY(v);
        j.setStepSizeABC(v);
    }
}
