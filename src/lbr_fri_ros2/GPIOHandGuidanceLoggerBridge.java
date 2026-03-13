package lbr_fri_ros2;

public class GPIOHandGuidanceLoggerBridge extends AbstractSunriseStreamBridge {
    private static final int DEFAULT_PORT = 30011;

    private volatile boolean loggingActive = false;

    @Override
    protected int getDefaultPort() {
        return DEFAULT_PORT;
    }

    @Override
    protected String getPortSystemPropertyName() {
        return "gpio.handguidance.logger.port";
    }

    @Override
    protected String getPortEnvironmentVariable() {
        return "GPIO_HANDGUIDANCE_LOGGER_PORT";
    }

    @Override
    protected String getBridgeName() {
        return "GPIOHandGuidanceLoggerBridge";
    }

    @Override
    protected String getSampleLabel() {
        return "GPIO hand-guidance";
    }

    @Override
    protected boolean shouldStreamSamples() {
        return true;
    }

    @Override
    protected boolean isSessionActive() {
        return loggingActive;
    }

    @Override
    protected void onSessionStart() throws Exception {
        loggingActive = true;
    }

    @Override
    protected void onLoop() {
        // No motion lifecycle to monitor in the GPIO logger bridge.
    }

    @Override
    protected synchronized String stopSession(String reason) {
        loggingActive = false;
        return stopRecorder(reason);
    }

    public static void main(String[] args) {
        GPIOHandGuidanceLoggerBridge app = new GPIOHandGuidanceLoggerBridge();
        app.runApplication();
    }
}
