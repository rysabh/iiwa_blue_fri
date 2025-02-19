package application;


import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class package_data_collection extends RoboticsAPIApplication {
	@Inject
	private LBR lBR_iiwa_7_R800_1;
	
	@Inject
	@Named("four_suction_tool")
	private Tool suction_tool;
	
	private ObjectFrame pick_tcp;
    private IUserKeyBar robot_control_bar;
    
    private String[] GoalOrientation = {"0","30","60","90"};
    private String[] PackageType = {"1", "2", "3"};
    private String[] joint_rel_vel = {"0.1", "0.5", "0.9"};
    private String[] joint_rel_acc = {"0.0", "0.5", "0.9"};
    
    
    private double CurrentGoalOrientation = 0.0, CurrentJointRelVel = 0.1, CurrentJointRelAcc = 0.0;
    private int CurrentPackageType = 1;
    
    private String home, pick_location, pre_pick_location, post_pick_location, nominal_place_location;
    
    private Frame place_location;
    private boolean exit_flag;
    
	@Override
	public void initialize() {
		// initialize your application here
		home = "/P1";
		pre_pick_location = "/P2";
		post_pick_location = "/P3";
		nominal_place_location = "/P4";
		
		exit_flag = false;
		
		
	}
	
	public void request_user_config() {
		// send period
		int id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION, 
				"Select the Goal Orientation for the current trial:", 
				GoalOrientation
		);
		CurrentGoalOrientation = Math.toRadians(Double.valueOf(GoalOrientation[id]));
		System.out.println("Goal orientation set to: "+ String.valueOf(Math.toDegrees(CurrentGoalOrientation)));
		
		id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION, 
				"Select the Joint Rel Vel for the current trial:", 
				joint_rel_vel
		);
		CurrentJointRelVel = Math.toRadians(Double.valueOf(joint_rel_vel[id]));
		System.out.println("Joint Relative Velocity set to: "+ String.valueOf(CurrentJointRelVel));
		
		id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION, 
				"Select the Joint Rel Acc for the current trial:", 
				joint_rel_acc
		);
		CurrentJointRelAcc = Math.toRadians(Double.valueOf(joint_rel_acc[id]));
		System.out.println("Joint Relative Acceleration set to: "+ String.valueOf(CurrentJointRelAcc));
		
		
		id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION, 
				"Select the Package Type for the current trial:", 
				PackageType
		);
		CurrentPackageType = Integer.valueOf(PackageType[id]);
		System.out.println("Executing Trial with Package Type: "+ String.valueOf(CurrentPackageType));
		
		return;
	}
	
	private DataRecorder initDataRecorder() {
        final String fmt = "yyyy-MM-dd_HH-mm-ss";
        final String timestamp = new SimpleDateFormat(fmt).format(new Date());
        final String fname = String.format("Package_"+String.valueOf(CurrentPackageType)+"_Vel_"+String.valueOf(CurrentJointRelVel)+ "_Acc_"+String.valueOf(CurrentJointRelAcc)+"_%s.log", timestamp);

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
	
	private void performPick(){
		System.out.println("Performing Pick");
		
		return;
	}
	
	private void beginDataCollection(){
		
		System.out.println("Executing Data Collection Sequence");
		
		
		return;
	}
	
	IUserKeyListener pick_event = new IUserKeyListener() {
        @Override
        public void onKeyEvent(IUserKey key, UserKeyEvent event) {
            if(event == UserKeyEvent.KeyDown) {
                performPick();
            }
        }
    };
    
    private void reset(){
    	System.out.println("Performing Reset");
    	return;
    }
    
    private void exit(){
    	exit_flag = true;
    }
    
    
    IUserKeyListener reset_event = new IUserKeyListener() {
        @Override
        public void onKeyEvent(IUserKey key, UserKeyEvent event) {
            if(event == UserKeyEvent.KeyDown) {
                reset();
            }
        }
    };
    
    
    IUserKeyListener data_collection_event = new IUserKeyListener() {
        @Override
        public void onKeyEvent(IUserKey key, UserKeyEvent event) {
            if(event == UserKeyEvent.KeyDown) {
            	beginDataCollection();
            }
        }
    };
    
    
    IUserKeyListener exit_event = new IUserKeyListener() {
        @Override
        public void onKeyEvent(IUserKey key, UserKeyEvent event) {
            if(event == UserKeyEvent.KeyDown) {
            	exit();
            }
        }
    };
    
    
    
	
    
	
	
	@Override
	public void run() {
		// your application execution starts here
		request_user_config();
		robot_control_bar = getApplicationUI().createUserKeyBar("Action Control Bar");
        robot_control_bar.addUserKey(0, pick_event, true).setText(UserKeyAlignment.TopLeft, "Begin Pick");
        robot_control_bar.addUserKey(1, data_collection_event, true).setText(UserKeyAlignment.TopLeft, "Collect Data");
        robot_control_bar.addUserKey(2, reset_event, true).setText(UserKeyAlignment.TopLeft, "Reset");
        robot_control_bar.addUserKey(3, exit_event, true).setText(UserKeyAlignment.TopLeft, "Exit");
        robot_control_bar.publish();
        suction_tool.attachTo(lBR_iiwa_7_R800_1.getFlange());
        pick_tcp = suction_tool.getFrame("/pick_tcp");
        
        System.out.println("Moving the Robot Home");
        try {
            pick_tcp.move(ptp(getApplicationData().getFrame(home)).setJointVelocityRel(0.1));
        } catch (Exception e) {
            System.out.println("Could not move to home due to");
            System.out.println(e.getMessage());
        }
        
        
        
        
		while(exit_flag){
			
		}
		
		
	}
}