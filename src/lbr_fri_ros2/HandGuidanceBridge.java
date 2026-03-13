package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;

public class HandGuidanceBridge extends AbstractSunriseStreamBridge {
    private static final int DEFAULT_PORT = 30010;

    private volatile boolean sessionActive = false;
    private IMotionContainer activeMotion;

    @Override
    protected int getDefaultPort() {
        return DEFAULT_PORT;
    }

    @Override
    protected String getPortSystemPropertyName() {
        return "handguidance.bridge.port";
    }

    @Override
    protected String getPortEnvironmentVariable() {
        return "HANDGUIDANCE_BRIDGE_PORT";
    }

    @Override
    protected String getBridgeName() {
        return "HandGuidanceBridge";
    }

    @Override
    protected String getSampleLabel() {
        return "hand-guidance";
    }

    @Override
    protected boolean shouldStreamSamples() {
        return sessionActive;
    }

    @Override
    protected boolean isSessionActive() {
        return sessionActive;
    }

    @Override
    protected void onSessionStart() throws Exception {
        lbr.setESMState("1");
        activeMotion = lbr.moveAsync(handGuiding());
        sessionActive = true;
    }

    @Override
    protected void onLoop() {
        IMotionContainer motion = activeMotion;
        if (motion == null || !motion.isFinished()) {
            return;
        }

        String reason = "motion_finished";
        try {
            if (motion.hasError()) {
                reason = "motion_error";
                getLogger().warn("Hand-guidance motion ended with error: " + motion.getErrorMessage());
            } else {
                ExecutionState state = motion.getState();
                if (state != null) {
                    reason = "motion_" + state.name().toLowerCase();
                }
            }
        } catch (Exception exc) {
            getLogger().warn("Unable to inspect motion completion state: " + exc.getMessage());
        }

        String finalFileName = stopSession(reason);
    }

    @Override
    protected synchronized String stopSession(String reason) {
        IMotionContainer motion = activeMotion;
        activeMotion = null;
        sessionActive = false;

        if (motion != null) {
            try {
                if (!motion.isFinished()) {
                    motion.cancel();
                }
            } catch (Exception exc) {
                getLogger().warn("Failed to cancel hand-guidance motion: " + exc.getMessage());
            }

            try {
                motion.await(5, TimeUnit.SECONDS);
            } catch (TimeoutException exc) {
                getLogger().warn("Timed out waiting for hand-guidance motion to finish.");
            }
        }

        return stopRecorder(reason);
    }

    public static void main(String[] args) {
        HandGuidanceBridge app = new HandGuidanceBridge();
        app.runApplication();
    }
}
