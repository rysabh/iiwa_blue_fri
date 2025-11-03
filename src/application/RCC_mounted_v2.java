package application;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class RCC_mounted_v2 extends RoboticsAPIApplication {
    @Inject
    private LBR lBR_iiwa_7_R800_1;
    
    @Inject
	@Named("BNC")  // Used for collecting SERL Demonstrations
	private Tool peg;
    private ObjectFrame peg_tcp;
    
    private IUserKeyBar robot_control_bar;
    private CartesianImpedanceControlMode impedanceControlmode;
    
    private boolean recorder_active = true;  // Changed to true by default
    
    public String shared_folder_add = "//192.168.10.190/kuka_data_collection/";
    
    @Override
    public void initialize() {
    	peg.attachTo(lBR_iiwa_7_R800_1.getFlange());
        peg_tcp = peg.getFrame("/TCP");
        impedanceControlmode = new CartesianImpedanceControlMode();
        impedanceControlmode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(100);
        impedanceControlmode.parametrize(CartDOF.Z).setStiffness(100);
//        impedanceControlmode.parametrize(CartDOF.ROT).setStiffness(0.2);
        impedanceControlmode.parametrize(CartDOF.A).setStiffness(0.1);
        impedanceControlmode.parametrize(CartDOF.B).setStiffness(0.1);
        impedanceControlmode.parametrize(CartDOF.C).setStiffness(0.1);
        impedanceControlmode.parametrize(CartDOF.ALL).setDamping(0.7);
        impedanceControlmode.setSpringPosition(peg_tcp);
    }
    
    private DataRecorder initDataRecorder() {
        final String fmt = "yyyy-MM-dd_HH-mm-ss";
        final String timestamp = new SimpleDateFormat(fmt).format(new Date());
        final String fname = String.format("RCC_compliance_data_collection_%s.log", timestamp);

        final long timeout = 360;  // 12 minutes
        final int sampleRate = 5;
        
        

        DataRecorder recorder = new DataRecorder(fname, timeout, TimeUnit.SECONDS, sampleRate);
        
        recorder.addCartesianForce(lBR_iiwa_7_R800_1.getFlange(), World.Current.getRootFrame())
                .addCartesianTorque(lBR_iiwa_7_R800_1.getFlange(), World.Current.getRootFrame())
                .addCurrentCartesianPositionXYZ(lBR_iiwa_7_R800_1.getFlange(), World.Current.getRootFrame())
                .addCommandedCartesianPositionXYZ(lBR_iiwa_7_R800_1.getFlange(), World.Current.getRootFrame())
                .addCurrentJointPosition(lBR_iiwa_7_R800_1, AngleUnit.Degree);
        recorder.enable();
        return recorder;
    }
    
    IUserKeyListener stoprecording_event = new IUserKeyListener() {
        @Override
        public void onKeyEvent(IUserKey key, UserKeyEvent event) {
            if(event == UserKeyEvent.KeyDown) {
                recorder_active = false;
                System.out.println("Stop Recording Now");
            }
        }
    };

    @Override
    public void run() {
        peg.attachTo(lBR_iiwa_7_R800_1.getFlange());
        peg_tcp = peg.getFrame("/Peg_Tip");

        robot_control_bar = getApplicationUI().createUserKeyBar("Action Control Bar");
        robot_control_bar.addUserKey(0, stoprecording_event, true).setText(UserKeyAlignment.TopLeft, "Stop Data");
        robot_control_bar.publish();
        
//        System.out.println("Moving the Robot to P1");
//        try {
//            peg_tcp.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.1));
//        } catch (Exception e) {
//            System.out.println("Could not move to P1 due to");
//            System.out.println(e.getMessage());
//        }
//        
//        System.out.println("Moving the Robot to P2");
//        try {
//            peg_tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.1).setMode(impedanceControlmode));
//        } catch (Exception e) {
//            System.out.println("Could not move to P2 due to");
//            System.out.println(e.getMessage());
//        }
        
//        System.out.println("Moving the Robot to P2");
//        try {
//            peg_tcp.move(ptp(getApplicationData().getFrame("/P3")).setJointVelocityRel(0.01).setMode(impedanceControlmode));
//        } catch (Exception e) {
//            System.out.println("Could not move to P3 due to");
//            System.out.println(e.getMessage());
//        }
        
        DataRecorder recorder = initDataRecorder();
        recorder.startRecording();
        System.out.println("Recording Started");
        
        while(recorder_active) {
            try {
                peg_tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.01).setMode(impedanceControlmode));
            } catch (Exception e) {
                System.out.println("Could not move to P2 due to");
                System.out.println(e.getMessage());
            }
        }
        
        recorder.stopRecording();
        
        if (recorder.awaitFileAvailable(5, TimeUnit.SECONDS)) {
            System.out.println("File is available");
        }
    }
}