package lbr_fri_ros2;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

public abstract class AbstractSunriseStreamBridge extends RoboticsAPIApplication {
    private static final int SOCKET_TIMEOUT_MS = 200;
    private static final long SAMPLE_PERIOD_MS = 50L;
    private static final long COMMAND_LOOP_SLEEP_MS = 10L;
    private static final double FORCE_VALID_TOLERANCE_N = 10.0;
    private static final double TORQUE_VALID_TOLERANCE_NM = 2.0;
    private static final long INVALID_SENSOR_WARN_INTERVAL_MS = 5000L;

    protected Controller kuka;
    protected LBR lbr;
    protected Tool tool;
    protected ObjectFrame toolTCP;
    protected volatile boolean running = true;
    protected DataRecorder activeRecorder;
    protected String currentLogFileName = "-";

    private int port;
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private BufferedReader clientReader;
    private BufferedWriter clientWriter;
    private volatile boolean clientConnected = false;
    private int clientConnectionId = 0;
    private Thread commandReaderThread;
    private long lastSampleTimestampMs = 0L;
    private long lastInvalidSensorWarnMs = 0L;
    private long recorderSequence = 0L;

    @Override
    public void initialize() {
        kuka = getController("KUKA_Sunrise_Cabinet_1");
        lbr = (LBR) getDevice(kuka, "LBR_iiwa_7_R800_1");
        tool = getApplicationData().createFromTemplate("umiGripper");
        tool.attachTo(lbr.getFlange());
        toolTCP = tool.getFrame("/umiGripper_TCP");
        port = resolvePort();
        getLogger().info(getBridgeName() + " port set to " + port);
    }

    private int resolvePort() {
        String propertyValue = System.getProperty(getPortSystemPropertyName());
        if (propertyValue == null || propertyValue.length() == 0) {
            propertyValue = System.getenv(getPortEnvironmentVariable());
        }

        if (propertyValue == null || propertyValue.length() == 0) {
            return getDefaultPort();
        }

        try {
            return Integer.parseInt(propertyValue);
        } catch (NumberFormatException exc) {
            getLogger().warn(
                    "Invalid " + getBridgeName() + " port '" + propertyValue + "', falling back to "
                            + getDefaultPort());
            return getDefaultPort();
        }
    }

    protected DataRecorder createDataRecorder() {
        final String fmt = "yyyy-MM-dd_HH-mm-ss_SSS";
        final String timestamp = new SimpleDateFormat(fmt).format(new Date());
        final String fileName = String.format("%s_%04d.log", timestamp,
                Long.valueOf(nextRecorderSequence()));

        final long timeout = 120;
        final int sampleRateMs = 50;

        DataRecorder recorder = new DataRecorder(fileName, timeout, TimeUnit.SECONDS, sampleRateMs)
                .addInternalJointTorque(lbr)
                .addCartesianForce(lbr.getFlange(), null)
                .addCartesianTorque(lbr.getFlange(), null)
                .addCurrentJointPosition(lbr, AngleUnit.Degree);
        recorder.enable();
        currentLogFileName = fileName;
        return recorder;
    }

    @Override
    public void run() {
        try {
            openServerSocket();
            while (running) {
                acceptAndServeClient();
                onLoop();
            }
        } finally {
            cleanupClient();
            cleanupServerSocket();
            stopSession("application_exit");
        }
    }

    private void openServerSocket() {
        try {
            serverSocket = new ServerSocket(port);
            serverSocket.setSoTimeout(SOCKET_TIMEOUT_MS);
            getLogger().info(getBridgeName() + " listening on port " + port);
        } catch (IOException exc) {
            throw new RuntimeException("Failed to open " + getBridgeName() + " server socket", exc);
        }
    }

    private void acceptAndServeClient() {
        if (serverSocket == null) {
            return;
        }

        try {
            Socket socket = serverSocket.accept();
            configureClient(socket);
            serveClient();
        } catch (SocketTimeoutException exc) {
            // Periodically break out to check session state and shutdown.
        } catch (IOException exc) {
            if (running) {
                getLogger().error("Client connection failure: " + exc.getMessage());
            }
            cleanupClient();
        }
    }

    private void configureClient(Socket socket) throws IOException {
        cleanupClient();
        clientSocket = socket;
        clientSocket.setSoTimeout(0);
        clientSocket.setTcpNoDelay(true);
        clientReader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream(), "UTF-8"));
        clientWriter = new BufferedWriter(new OutputStreamWriter(clientSocket.getOutputStream(), "UTF-8"));
        markClientConnected();
        startCommandReader(getClientConnectionId(), clientReader);
        getLogger().info("Client connected from " + clientSocket.getInetAddress().getHostAddress());
    }

    private void serveClient() {
        while (running && isClientConnected()) {
            onLoop();
            maybeSendSample();

            try {
                Thread.sleep(COMMAND_LOOP_SLEEP_MS);
            } catch (InterruptedException exc) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        cleanupClient();
    }

    private void handleCommand(String rawCommand) {
        if (rawCommand == null || rawCommand.length() == 0) {
            return;
        }

        String command = rawCommand.toUpperCase();
        getLogger().info(getBridgeName() + " command: " + command);

        if ("PING".equals(command)) {
            sendOk("PING");
        } else if ("STATUS".equals(command)) {
            sendOk("STATUS," + getSessionState() + "," + sanitizeToken(currentLogFileName));
        } else if ("START".equals(command)) {
            handleStartCommand();
        } else if ("STOP".equals(command)) {
            handleStopCommand("remote_stop");
        } else {
            sendErr("UNKNOWN_COMMAND," + sanitizeToken(rawCommand));
        }
    }

    private synchronized void handleStartCommand() {
        if (isSessionActive()) {
            sendOk("START,active," + sanitizeToken(currentLogFileName) + ",already_active");
            return;
        }

        try {
            activeRecorder = createDataRecorder();
            activeRecorder.startRecording();
            onSessionStart();
            lastSampleTimestampMs = 0L;

            getLogger().info(getBridgeName() + " started session.");
            sendOk("START,active," + sanitizeToken(currentLogFileName));
        } catch (Exception exc) {
            getLogger().error("Failed to start " + getBridgeName() + " session: " + exc.getMessage());
            stopSession("start_failed");
            sendErr("START," + sanitizeToken(exc.getMessage()));
        }
    }

    protected synchronized void handleStopCommand(String reason) {
        if (!isSessionActive() && activeRecorder == null) {
            sendOk("STOP,idle," + sanitizeToken(currentLogFileName) + ",already_idle");
            return;
        }

        String finalFileName = stopSession(reason);
        sendOk("STOP,idle," + sanitizeToken(finalFileName));
    }

    protected synchronized String stopRecorder(String reason) {
        DataRecorder recorder = activeRecorder;
        String finalFileName = currentLogFileName;
        activeRecorder = null;

        if (recorder != null) {
            try {
                recorder.stopRecording();
            } catch (Exception exc) {
                getLogger().warn("Failed to stop DataRecorder cleanly: " + exc.getMessage());
            }

            try {
                recorder.awaitFileAvailable(5, TimeUnit.SECONDS);
            } catch (Exception exc) {
                getLogger().warn("Recorder file availability wait failed: " + exc.getMessage());
            }

            try {
                finalFileName = recorder.getFileName();
            } catch (Exception exc) {
                getLogger().warn("Failed to read recorder filename: " + exc.getMessage());
            }
        }

        currentLogFileName = finalFileName == null ? "-" : finalFileName;
        getLogger().info(getBridgeName() + " session stopped: " + reason + " file=" + currentLogFileName);
        return currentLogFileName;
    }

    private synchronized long nextRecorderSequence() {
        long sequence = recorderSequence;
        recorderSequence = recorderSequence + 1L;
        return sequence;
    }

    private void maybeSendSample() {
        if (!shouldStreamSamples() || clientWriter == null) {
            return;
        }

        long nowMs = System.currentTimeMillis();
        if (nowMs - lastSampleTimestampMs < SAMPLE_PERIOD_MS) {
            return;
        }
        lastSampleTimestampMs = nowMs;

        String sampleLine = buildSampleLine(nowMs * 1000000L);
        if (sampleLine != null) {
            sendLine(sampleLine);
        }
    }

    private synchronized String buildSampleLine(long timestampNs) {
        try {
            JointPosition jointPosition = lbr.getCurrentJointPosition();
            TorqueSensorData torqueData = lbr.getExternalTorque();
            Frame tcpFrame = lbr.getCurrentCartesianPosition(toolTCP);
            ForceSensorData forceData = lbr.getExternalForceTorque(toolTCP, null);

            double[] joints = jointPosition.get();
            double[] torques = torqueData.getTorqueValues();
            Vector forceVector = forceData.getForce();
            Vector torqueVector = forceData.getTorque();

            boolean forceValid = forceData.isForceValid(FORCE_VALID_TOLERANCE_N);
            boolean torqueValid = forceData.isTorqueValid(TORQUE_VALID_TOLERANCE_NM);
            if ((!forceValid || !torqueValid)
                    && (System.currentTimeMillis() - lastInvalidSensorWarnMs) > INVALID_SENSOR_WARN_INTERVAL_MS) {
                lastInvalidSensorWarnMs = System.currentTimeMillis();
                getLogger().warn("Force/Torque sample marked invalid by Sunrise, sending NaN values.");
            }

            StringBuilder builder = new StringBuilder();
            builder.append("SAMPLE");
            appendLong(builder, timestampNs);
            appendDoubleArray(builder, joints);
            appendDoubleArray(builder, torques);
            appendDouble(builder, tcpFrame.getX());
            appendDouble(builder, tcpFrame.getY());
            appendDouble(builder, tcpFrame.getZ());
            appendDouble(builder, tcpFrame.getAlphaRad());
            appendDouble(builder, tcpFrame.getBetaRad());
            appendDouble(builder, tcpFrame.getGammaRad());

            if (forceValid) {
                appendDouble(builder, forceVector.getX());
                appendDouble(builder, forceVector.getY());
                appendDouble(builder, forceVector.getZ());
            } else {
                appendNaNs(builder, 3);
            }

            if (torqueValid) {
                appendDouble(builder, torqueVector.getX());
                appendDouble(builder, torqueVector.getY());
                appendDouble(builder, torqueVector.getZ());
            } else {
                appendNaNs(builder, 3);
            }

            return builder.toString();
        } catch (Exception exc) {
            getLogger().warn("Failed to build " + getSampleLabel() + " sample: " + exc.getMessage());
            return null;
        }
    }

    private void appendDoubleArray(StringBuilder builder, double[] values) {
        int idx;
        for (idx = 0; idx < values.length; idx++) {
            appendDouble(builder, values[idx]);
        }
    }

    private void appendNaNs(StringBuilder builder, int count) {
        int idx;
        for (idx = 0; idx < count; idx++) {
            appendDouble(builder, Double.NaN);
        }
    }

    private void appendLong(StringBuilder builder, long value) {
        builder.append(',').append(value);
    }

    private void appendDouble(StringBuilder builder, double value) {
        builder.append(',').append(Double.toString(value));
    }

    protected String getSessionState() {
        return isSessionActive() ? "active" : "idle";
    }

    protected synchronized void sendOk(String payload) {
        sendLine("OK," + payload);
    }

    protected synchronized void sendErr(String payload) {
        sendLine("ERR," + payload);
    }

    protected synchronized void sendLine(String line) {
        if (clientWriter == null) {
            return;
        }

        try {
            clientWriter.write(line);
            clientWriter.newLine();
            clientWriter.flush();
        } catch (IOException exc) {
            if (running) {
                getLogger().warn("Failed to send line to client: " + exc.getMessage());
            }
            cleanupClient();
        }
    }

    protected String sanitizeToken(String value) {
        if (value == null || value.length() == 0) {
            return "-";
        }
        return value.replace(',', '_').replace('\n', '_').replace('\r', '_');
    }

    private synchronized void markClientConnected() {
        clientConnectionId = clientConnectionId + 1;
        clientConnected = true;
    }

    private synchronized int getClientConnectionId() {
        return clientConnectionId;
    }

    private synchronized boolean isClientConnected() {
        return clientConnected && clientSocket != null && !clientSocket.isClosed();
    }

    private synchronized boolean isCurrentClientConnection(int connectionId) {
        return clientConnected && connectionId == clientConnectionId;
    }

    private synchronized void markClientDisconnected(int connectionId) {
        if (connectionId == clientConnectionId) {
            clientConnected = false;
        }
    }

    private void startCommandReader(final int connectionId, final BufferedReader reader) {
        commandReaderThread = new Thread(new Runnable() {
            public void run() {
                runCommandReader(connectionId, reader);
            }
        }, getBridgeName() + "-command-reader");
        commandReaderThread.setDaemon(true);
        commandReaderThread.start();
    }

    private void runCommandReader(int connectionId, BufferedReader reader) {
        try {
            while (running && isCurrentClientConnection(connectionId) && reader != null) {
                String command = reader.readLine();
                if (command == null) {
                    break;
                }
                command = command.trim();
                if (command.length() != 0) {
                    handleCommand(command);
                }
            }
        } catch (IOException exc) {
            if (running && isCurrentClientConnection(connectionId)) {
                getLogger().warn("Client IO closed: " + exc.getMessage());
            }
        } finally {
            markClientDisconnected(connectionId);
        }
    }

    private void cleanupClient() {
        Thread readerThread;
        synchronized (this) {
            clientConnected = false;
            clientConnectionId = clientConnectionId + 1;
            readerThread = commandReaderThread;
            commandReaderThread = null;
        }

        if (clientReader != null) {
            try {
                clientReader.close();
            } catch (IOException exc) {
                // Ignore cleanup exception.
            }
        }
        if (clientWriter != null) {
            try {
                clientWriter.close();
            } catch (IOException exc) {
                // Ignore cleanup exception.
            }
        }
        if (clientSocket != null) {
            try {
                clientSocket.close();
            } catch (IOException exc) {
                // Ignore cleanup exception.
            }
        }

        clientReader = null;
        clientWriter = null;
        clientSocket = null;

        if (readerThread != null && readerThread != Thread.currentThread()) {
            try {
                readerThread.join(100);
            } catch (InterruptedException exc) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void cleanupServerSocket() {
        if (serverSocket != null) {
            try {
                serverSocket.close();
            } catch (IOException exc) {
                // Ignore cleanup exception.
            }
        }
        serverSocket = null;
    }

    @Override
    public void dispose() {
        running = false;
        cleanupClient();
        cleanupServerSocket();
        stopSession("dispose");
        super.dispose();
    }

    protected abstract int getDefaultPort();

    protected abstract String getPortSystemPropertyName();

    protected abstract String getPortEnvironmentVariable();

    protected abstract String getBridgeName();

    protected abstract String getSampleLabel();

    protected abstract boolean shouldStreamSamples();

    protected abstract boolean isSessionActive();

    protected abstract void onSessionStart() throws Exception;

    protected abstract void onLoop();

    protected abstract String stopSession(String reason);
}
