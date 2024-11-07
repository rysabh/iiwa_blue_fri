package lbr_fri_ros2;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;


import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.TimeUnit;


public class handGuidance extends RoboticsAPIApplication {

    private Controller kuka;
    private LBR lbr;
 


    @Override
    public void initialize() {
        kuka = getController("KUKA_Sunrise_Cabinet_1");
        lbr = (LBR) getDevice(kuka, "LBR_iiwa_7_R800_1");
    }

    private DataRecorder initDataRecorder() {
        // Set filename with timestamp
        final String fmt = "yyyy-MM-dd_HH-mm-ss";
        final String timestamp = new SimpleDateFormat(fmt).format(new Date());
        final String fname = String.format("%s.log", timestamp);

        // Recorder intrinsic settings
        final long timeout = 120; // seconds
        final int sampleRate = 333; // ms

        // Data recorder builder
        DataRecorder recorder = new DataRecorder(fname, timeout, TimeUnit.SECONDS, sampleRate)
                .addInternalJointTorque(lbr)
                .addCartesianForce(lbr.getFlange(), null)
                .addCartesianTorque(lbr.getFlange(), null)
                .addCurrentJointPosition(lbr, AngleUnit.Degree);
        recorder.enable();
        return recorder;
    }

    @Override
    public void run() {
        // Create the data recorder object
        final DataRecorder recorder = initDataRecorder();
        recorder.startRecording();

        // Enable hand guiding
        lbr.setESMState("1");
        getLogger().info("Now HandGuiding");
        lbr.move(handGuiding());
        // Stop recording and wait for the file to be available
        recorder.stopRecording();
        recorder.awaitFileAvailable(3, TimeUnit.SECONDS);

        // Log the filename for future reference
        getLogger().info("Logged data filename: " + recorder.getFileName());
    }

    @Override
    public void dispose() {
        lbr.setESMState("1");
        super.dispose();
    }

    public static void main(String[] args) {
        handGuidance app = new handGuidance();
        app.runApplication();
    }
}