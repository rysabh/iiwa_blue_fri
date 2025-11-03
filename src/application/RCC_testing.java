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
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.persistenceModel.templateModel.FrameTemplate.Transformation;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;


public class RCC_testing extends RoboticsAPIApplication {
    @Inject
    private LBR lBR_iiwa_14_R820_1;
//  @Named("CrossPeg")  // Used for getting inserted pose for ICP method
    
    @Inject
	@Named("RectPegTwoCameras")  // Used for collecting SERL Demonstrations
	private Tool peg;
    
    private ObjectFrame peg_tcp;
    private IUserKeyBar robot_control_bar;
//    private CartesianImpedanceControlMode impedanceControlmode;
    private PositionControlMode positionControlmode; 
    private boolean recorder_active = true;  // Changed to true by default
    
    public String shared_folder_add = "//192.168.10.190/kuka_data_collection/";
    
    @Override
    public void initialize() {
//        impedanceControlmode = new CartesianImpedanceControlMode();
//        impedanceControlmode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(100);
//        impedanceControlmode.parametrize(CartDOF.Z).setStiffness(100);
//        impedanceControlmode.parametrize(CartDOF.ROT).setStiffness(1);
//        impedanceControlmode.parametrize(CartDOF.ALL).setDamping(0.7);
    	positionControlmode = new PositionControlMode(); 
    }
    
    private DataRecorder initDataRecorder() {
        final String fmt = "yyyy-MM-dd_HH-mm-ss";
        final String timestamp = new SimpleDateFormat(fmt).format(new Date());
        final String fname = String.format("RCC_test_May2_%s.log", timestamp);

        final long timeout = 360;  // 12 minutes
        final int sampleRate = 5;
        
        

        DataRecorder recorder = new DataRecorder(fname, timeout, TimeUnit.SECONDS, sampleRate);
        
        recorder.addCartesianForce(lBR_iiwa_14_R820_1.getFlange(), World.Current.getRootFrame())
                .addCartesianTorque(lBR_iiwa_14_R820_1.getFlange(), World.Current.getRootFrame())
                .addCurrentCartesianPositionXYZ(lBR_iiwa_14_R820_1.getFlange(), World.Current.getRootFrame())
                .addCommandedCartesianPositionXYZ(lBR_iiwa_14_R820_1.getFlange(), World.Current.getRootFrame())
                .addCurrentJointPosition(lBR_iiwa_14_R820_1, AngleUnit.Degree);
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
        peg.attachTo(lBR_iiwa_14_R820_1.getFlange());
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
        
//        while(recorder_active) {
//            try {
//                peg_tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.01).setMode(impedanceControlmode));
//            } catch (Exception e) {
//                System.out.println("Could not move to P2 due to");
//                System.out.println(e.getMessage());
//            }
//        }
        
        System.out.println("Beginning of movements"); 
        
        
        Frame insert_pose = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange()).copy();
        insert_pose.setX(768.81);
        insert_pose.setY(-87.81);
        insert_pose.setZ(211.99);
        insert_pose.setAlphaRad(-134.45*Math.PI/180.0);
        insert_pose.setBetaRad(0.45*Math.PI/180.0);
        insert_pose.setGammaRad(-179.15*Math.PI/180.0); 
		
		com.kuka.roboticsAPI.geometricModel.math.Transformation above_offset = 
			    com.kuka.roboticsAPI.geometricModel.math.Transformation.ofDeg(0,0,-50,0,0,0);
		Frame above_insert_pose = insert_pose.copy().transform(peg_tcp, above_offset);
        
        // nominal insertion 
        System.out.println("Nominal insertion"); 
        lBR_iiwa_14_R820_1.getFlange().move(lin(above_insert_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
        lBR_iiwa_14_R820_1.getFlange().move(lin(insert_pose).setJointVelocityRel(0.01).setMode(positionControlmode));
        lBR_iiwa_14_R820_1.getFlange().move(lin(above_insert_pose).setJointVelocityRel(0.05).setMode(positionControlmode));

//        // translated insertion - 1mm increments up to 5mm 
        for (int i=0; i<10; i++) {
		
        	com.kuka.roboticsAPI.geometricModel.math.Transformation translation_offset = com.kuka.roboticsAPI.geometricModel.math.Transformation.ofDeg(i,0,0,0,0,0);
            Frame offset_insert_pose = insert_pose.copy().transform(peg_tcp, translation_offset);  
            Frame offset_above_pose = insert_pose.copy().transform(peg_tcp, above_offset);  
            
            System.out.println("Translation = " + i + " mm"); 
            
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_above_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_insert_pose).setJointVelocityRel(0.01).setMode(positionControlmode));
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_above_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
		}
        
        // tilted insertion - 1deg increments up to 5deg 
        for (int i=0; i<11; i++) 
		{
        	com.kuka.roboticsAPI.geometricModel.math.Transformation tilted_offset = 
    			    com.kuka.roboticsAPI.geometricModel.math.Transformation.ofDeg(0,0,0,0,0,i);
            Frame offset_insert_pose = insert_pose.copy().transform(peg_tcp, tilted_offset);  
            Frame offset_above_pose= insert_pose.copy().transform(peg_tcp, above_offset);  
            
            System.out.println("Translation = " + i + " mm"); 
            
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_above_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_insert_pose).setJointVelocityRel(0.01).setMode(positionControlmode));
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_above_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
		}
        
        // twisted insertion - 1 deg increments up to 5 deg 
        for (int i=0; i<15; i++) 
		{
        	com.kuka.roboticsAPI.geometricModel.math.Transformation offset = 
    			    com.kuka.roboticsAPI.geometricModel.math.Transformation.ofDeg(0,0,0,i,0,0);
            Frame offset_insert_pose = insert_pose.copy().transform(peg_tcp, offset);  
            Frame offset_above_pose= insert_pose.copy().transform(peg_tcp, above_offset);  
            
            System.out.println("Translation = " + i + " mm"); 
            
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_above_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_insert_pose).setJointVelocityRel(0.01).setMode(positionControlmode));
            lBR_iiwa_14_R820_1.getFlange().move(lin(offset_above_pose).setJointVelocityRel(0.05).setMode(positionControlmode));
		}
        
        System.out.println("End of movements"); 
        
        recorder.stopRecording();
        
        if (recorder.awaitFileAvailable(5, TimeUnit.SECONDS)) {
            System.out.println("File is available");
        }
    }
}