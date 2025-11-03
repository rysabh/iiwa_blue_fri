package application;


import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.sql.Time;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.SafetyStopType;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.params.RobotParameterSet;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.executionModel.ExecutionState;

import sun.awt.SunHints.Value;
import sun.nio.ch.SocketOpts.IP.TCP;
import utilities.ReadWriteUtilities;
import utilities.TcpipComm;

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
public class ROS2Client extends RoboticsAPIApplication {
	@Inject
	private LBR lBR_iiwa_14_R820_1;
	@Inject
	@Named("CrossPeg")
	private Tool peg;
	
	private ObjectFrame peg_tcp, site_1, site_2, site_3, site_4;

	//Controller Definitions
	private CartesianImpedanceControlMode impedanceControlmode;
//	private JointImpedanceControlMode jointImpedanceControl;
	private PositionControlMode positionControlMode; 
	
	private String shared_folder_path;
	
	private double robSpeed;
	
	public String client_ip_address = "192.168.10.103";	//Alternate System
	private int clientPort;
	public TcpipComm TCP_IP_Handler;
	
	private String control_mode;
	public double time_to_collect_obs_sec;
	public double pi_z_threshold;
	
	public IMotionContainer cartesian_motion;
	public ForceSensorData force_val;
	ArrayList<double[]> observation_data = new ArrayList<double[]>();
	ArrayList<double[]> current_joint_position = new ArrayList<double[]>();
	

	
	@Override
	public void initialize() {
		// initialize your application here

		control_mode = "impedance";
		//Initializing the Peg TCP
		peg.attachTo(lBR_iiwa_14_R820_1.getFlange());
		peg_tcp = peg.getFrame("/Peg_Tip");
		
		site_1 = peg.getFrame("/site_1");
		site_2 = peg.getFrame("/site_2");
		site_3 = peg.getFrame("/site_3");
		site_4 = peg.getFrame("/site_4");
		
		time_to_collect_obs_sec = 1000.0*60.0;  // 20 minutes
		pi_z_threshold = 106; // 110 corresponds to the surface
		
		impedanceControlmode = new CartesianImpedanceControlMode();
		impedanceControlmode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(500);
		impedanceControlmode.parametrize(CartDOF.Z).setStiffness(500);
		impedanceControlmode.parametrize(CartDOF.ROT).setStiffness(100);
		impedanceControlmode.parametrize(CartDOF.ALL).setDamping(0.7);
		
		//Initializing Position Control Mode
		positionControlMode = new PositionControlMode();
		
		
		//Initialize the shared folder path
		shared_folder_path = "//192.168.10.100/PoseAccDataCollection";
		robSpeed = 0.1;
		
		client_ip_address = "192.168.10.150";	// 101:dhanush, 190:omey, 109:aero, Boeing: 103
		clientPort = 30015;
		
		
	}
	
	
	
	private DataRecorder initDataRecorder() {
        // Set filename with timestamp
        final String fmt = "yyyy-MM-dd_HH-mm-ss";
        final String timestamp = new SimpleDateFormat(fmt).format(new Date());
        final String fname = String.format("Exp1_%s.log", timestamp);

        // Recorder intrinsic settings
        final long timeout = 120; // seconds
        final int sampleRate = 10; // ms

        // Data recorder builder
        DataRecorder recorder = new DataRecorder(fname, timeout, TimeUnit.SECONDS, sampleRate)
                .addCartesianForce(lBR_iiwa_14_R820_1.getFlange(), World.Current.getRootFrame())
                .addCartesianTorque(lBR_iiwa_14_R820_1.getFlange(), World.Current.getRootFrame())
                .addCurrentCartesianPositionXYZ(peg_tcp, World.Current.getRootFrame())
                .addCommandedCartesianPositionXYZ(peg_tcp, World.Current.getRootFrame());
        recorder.enable();

        return recorder;
    }
	
	public void recordCartesianConfig(){
		return;
				
	}

	// NOTE(dhanush): You commented, since this method was hogging time (hypothesis)
//	public boolean partialInsertion(){
//		boolean partial_insertion = false;
//
//		Frame s1 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_1).copy();
//		Frame s2 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_2).copy();
//		Frame s3 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_3).copy();
//		Frame s4 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_4).copy();
//
//		double z_threshold = this.pi_z_threshold;
//		
//		if((s1.getZ()<z_threshold) && (s2.getZ()<z_threshold) && (s3.getZ()<z_threshold) && (s4.getZ()<z_threshold)){
//			partial_insertion = true;
//			return partial_insertion;
//		}
//		
//		return partial_insertion;
//	}
	
	public boolean partialInsertion() {
	    double z_threshold = this.pi_z_threshold;
	    
	    // Get Z positions directly without copying the entire Frame
	    double z1 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_1).getZ();
	    if (z1 >= z_threshold) return false;  // Early exit
	    
	    double z2 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_2).getZ();
	    if (z2 >= z_threshold) return false;  // Early exit
	    
	    double z3 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_3).getZ();
	    if (z3 >= z_threshold) return false;  // Early exit
	    
	    double z4 = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(site_4).getZ();
	    if (z4 >= z_threshold) return false;  // Early exit
	    
	    return true;
	}
		
	public void executeSplineMotionFromCartesianConfig(ArrayList<double[]> cartesian_config, boolean record_data, boolean check_partial_insertion)
	{
		// NOTE: To define the frame - 
		////////////////////////////////////////////////////		
		 Frame F_in = new Frame();
		 F_in = lBR_iiwa_14_R820_1.getFlange().copyWithRedundancy();//  ...This is used to get the redundancy value for motion.
		////////////////////////////////////////////////////
		observation_data.clear();
		boolean partial_insertion = false;
		int data_count = 0;
		
		int N = cartesian_config.size();
		if (N != 0)
		{
			System.out.println("Executing spline motion from cartesian waypoints..."); 
			Frame F[] = new Frame[N];
			SPL[] path = new SPL[N];
			int idx = 0;
			for (double[] cartesian_state : cartesian_config) 
			{
				F[idx] = F_in.copy();
				F[idx].setX(cartesian_state[0]*1000);
	        	F[idx].setY(cartesian_state[1]*1000);
	        	F[idx].setZ(cartesian_state[2]*1000);
	        	F[idx].setAlphaRad(cartesian_state[3]);
	        	F[idx].setBetaRad(cartesian_state[4]);
	        	F[idx].setGammaRad(cartesian_state[5]);
	        	path[idx]=new SPL(F[idx]);
	        	idx++;
			}
			Spline curve = new Spline(path);
			try
			{	
				if(!record_data){
					if(this.control_mode.equals("impedance")){
						lBR_iiwa_14_R820_1.move(curve.setJointVelocityRel(robSpeed).setMode(impedanceControlmode));
					}
					else{
						lBR_iiwa_14_R820_1.move(curve.setJointVelocityRel(robSpeed).setMode(positionControlMode));
					}
				}
				else{
					System.out.println("recording data");
					if(this.control_mode.equals("impedance")){
						System.out.println("Executing under impedance control");
						cartesian_motion = lBR_iiwa_14_R820_1.moveAsync(curve.setJointVelocityRel(robSpeed).setMode(impedanceControlmode));
					}
					else{
						System.out.println("Executing under position control");
						cartesian_motion = lBR_iiwa_14_R820_1.moveAsync(curve.setJointVelocityRel(robSpeed).setMode(positionControlMode));
					}
				}
				
						
				
			}
        	catch(Exception e)
        	{
				System.out.println(" Motion is not possible"); 
        	}
			
			// NOTE : to prevent premature start of timer/data collection
			while (cartesian_motion.getState() != ExecutionState.Executing) { }
			
			if(record_data){
				long start_time = System.currentTimeMillis();
				while(!cartesian_motion.isFinished()){
					long current_time = System.currentTimeMillis();
					data_count++;
					if((current_time - start_time)>(this.time_to_collect_obs_sec * 1000.0)){
						System.out.println("System Time: "+String.valueOf(this.time_to_collect_obs_sec));
						System.out.println("Time: "+String.valueOf(current_time - start_time));
						System.out.println("Time for observation collection elapsed, quitting");
						cartesian_motion.cancel();
						// NOTE: below lines to be removed.
//						TCP_IP_Handler.sendStringResponse("time_elapsed\r\n");
//		    			TCP_IP_Handler.getStringRequest(); //Handshake
						break;
					}
		    		force_val = lBR_iiwa_14_R820_1.getExternalForceTorque(lBR_iiwa_14_R820_1.getFlange());
		    		Frame pose = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange());
		    		
		    		//Check for partial insertion
		    		// Note : reduced frequency of checking for partial insertion
		    		if(check_partial_insertion && data_count % 50 == 0){
		    			partial_insertion = partialInsertion();
		    			if(partial_insertion){
		    				System.out.println("Partial Insertion Detected, Canceling Motion");
			    			cartesian_motion.cancel();
			    			TCP_IP_Handler.sendStringResponse("partial_insertion\r\n");
			    			TCP_IP_Handler.getStringRequest(); //Handshake
			    			break;
		    			}
		    		}
		    		
		    		// NOTE: more efficient data saving
		    		double[] current_observation = new double[13];
		    		// Time and position - direct assignment
		    		current_observation[0] = (double)current_time;
		    		current_observation[1] = pose.getX()/1000;
		    		current_observation[2] = pose.getY()/1000;
		    		current_observation[3] = pose.getZ()/1000;

		    		// Angles - direct assignment
		    		current_observation[4] = pose.getAlphaRad();
		    		current_observation[5] = pose.getBetaRad();
		    		current_observation[6] = pose.getGammaRad();

		    		// Force and torque - direct from arrays
		    		double[] force_data = force_val.getForce().toArray();
		    		double[] torque_data = force_val.getTorque().toArray();
		    		for(int i = 0; i < 3; i++) {
		    		    current_observation[7 + i] = force_data[i];
		    		    current_observation[10 + i] = torque_data[i];
		    		}

		    		observation_data.add(current_observation);
					
		    	}
			}
			
		}
		else
		{
			System.out.println("0 waypoints found");
		}
		if(check_partial_insertion){
			if(!partial_insertion){
				System.out.println("Partial Insertion Not Detected");
//				cartesian_motion.cancel();
				TCP_IP_Handler.sendStringResponse("no_partial_insertion\r\n");
				TCP_IP_Handler.getStringRequest(); //Handshake
			}
		}
		System.out.println("Data Count: "+ String.valueOf(observation_data.size())+ "Loop Count: "+ String.valueOf(data_count));
		
		// Reset the time to collect observations
		this.time_to_collect_obs_sec = 1000.0 * 60.0;
		
	}
	
	private void setJointRelVel(double joint_vel){
		this.robSpeed = joint_vel;
	}
	
	private void moveRobot(ArrayList<double[]> frame, boolean linear){
		Frame current_position = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange()).copy();
		current_position.setX(frame.get(0)[0]*1000);
		current_position.setY(frame.get(0)[1]*1000);
		current_position.setZ(frame.get(0)[2]*1000);
		current_position.setAlphaRad(frame.get(0)[3]);
		current_position.setBetaRad(frame.get(0)[4]);
		current_position.setGammaRad(frame.get(0)[5]);
		
		if(this.control_mode.equals("impedance")){
			System.out.println("Executing under impedance control");
			if(linear){
				lBR_iiwa_14_R820_1.move(lin(current_position).setJointVelocityRel(robSpeed).setMode(impedanceControlmode));
			}
			else{
				lBR_iiwa_14_R820_1.move(ptp(current_position).setJointVelocityRel(robSpeed).setMode(impedanceControlmode));
			}
			
		}
		else{
			System.out.println("Executing under position control");
			if(linear){
				lBR_iiwa_14_R820_1.move(lin(current_position).setJointVelocityRel(robSpeed).setMode(positionControlMode));
			}
			else{
				lBR_iiwa_14_R820_1.move(ptp(current_position).setJointVelocityRel(robSpeed).setMode(positionControlMode));
			}
			
		}
		
	}
	
	private ArrayList<double[]> CreateCartesianTraj(ArrayList<String> FrameNames){
		
		int N = FrameNames.size();
		ArrayList<double[]> cartesian_traj= new ArrayList<double[]>();
		for(int i = 0; i<N; i++){
			double[] cartesian_pose = new double[6];
			String frame_name = FrameNames.get(i);
			Frame F = getApplicationData().getFrame(frame_name).copyWithRedundancy();
			cartesian_pose[0] = F.getX();
			cartesian_pose[1] = F.getY();
			cartesian_pose[2] = F.getZ();
			cartesian_pose[3] = F.getAlphaRad();
			cartesian_pose[4] = F.getBetaRad();
			cartesian_pose[5] = F.getGammaRad();
			
			cartesian_traj.add(cartesian_pose);
			
		}
		
		return cartesian_traj;
	}
	
	void move_to_joint_pose(ArrayList<double[]> joint_position){
		JointPosition position = new JointPosition(joint_position.get(0));
		ReadWriteUtilities.display2DListDouble(joint_position.get(0));
		try{
			lBR_iiwa_14_R820_1.move(ptp(position).setJointVelocityRel(0.1));
		}
		catch(Exception e)
    	{
			System.out.println("Joint Space Motion is not possible"); 
    	}
		
	}
	
	
	@Override
	public void run() {
		// your application execution starts here
		
		System.out.println("Establishing Connection with the Client");
		TCP_IP_Handler = new TcpipComm(client_ip_address, clientPort, "client");
		TCP_IP_Handler.establishConnection();		
				
		
		String command_msg, temp_msg;
		
		while(true){
			
			System.out.println("Waiting to receive command message from Server");
			command_msg = TCP_IP_Handler.getStringRequest();
			System.out.println("Received Msg: "+command_msg);
			
			if(command_msg.equals("close_connection")){
				break;
			}
			
			else if(command_msg.equals("execute_joint_spline")){
				System.out.println("Waiting for Joint Trajectory");
			}
			
			else if(command_msg.equals("execute_cartesian_spline")){
				System.out.println("Waiting for Cartesian Trajectory");
				ArrayList<double[]> cartesian_traj = TCP_IP_Handler.getDoubleNPArrRequest();
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
				
				System.out.println("Received Following Trajectory");
//				ReadWriteUtilities.display2DListDouble(cartesian_traj);
				String resp = TCP_IP_Handler.getStringRequest();
				System.out.println("Received the following: "+resp);
				boolean check_partial_insertion = false;
				if(resp.equals("check_partial_insertion")){
					check_partial_insertion = true;
				}
				executeSplineMotionFromCartesianConfig(cartesian_traj, true, check_partial_insertion);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
				
				
			}
			
			else if(command_msg.equals("change_ctrl_mode")){
				System.out.println("Waiting for Control Mode Params");
				TCP_IP_Handler.sendStringResponse("s"+"\r\n");
				temp_msg = TCP_IP_Handler.getStringRequest();
				System.out.println("Setting the control mode to: "+ temp_msg);
				this.control_mode = temp_msg;
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("set_impedance_values")){
				System.out.println("Waiting for Control Mode Params");
				ArrayList<double[]> control_params = TCP_IP_Handler.getDoubleNPArrRequest();
				System.out.println("Received Following Cart Imp Params");
				ReadWriteUtilities.display2DListDouble(control_params);
				impedanceControlmode = new CartesianImpedanceControlMode();
				impedanceControlmode.parametrize(CartDOF.X).setStiffness(control_params.get(0)[0]);
				impedanceControlmode.parametrize(CartDOF.Y).setStiffness(control_params.get(0)[1]);
				impedanceControlmode.parametrize(CartDOF.Z).setStiffness(control_params.get(0)[2]);
				impedanceControlmode.parametrize(CartDOF.A).setStiffness(control_params.get(0)[3]);
				impedanceControlmode.parametrize(CartDOF.B).setStiffness(control_params.get(0)[4]);
				impedanceControlmode.parametrize(CartDOF.C).setStiffness(control_params.get(0)[5]);
				impedanceControlmode.parametrize(CartDOF.X).setDamping(control_params.get(0)[6]);
				impedanceControlmode.parametrize(CartDOF.Y).setDamping(control_params.get(0)[7]);
				impedanceControlmode.parametrize(CartDOF.Z).setDamping(control_params.get(0)[8]);
				impedanceControlmode.parametrize(CartDOF.A).setDamping(control_params.get(0)[9]);
				impedanceControlmode.parametrize(CartDOF.B).setDamping(control_params.get(0)[10]);
				impedanceControlmode.parametrize(CartDOF.C).setDamping(control_params.get(0)[11]);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("change_joint_rel_vel")){
				System.out.println("Waiting for Joint Velocity Params");
				TCP_IP_Handler.sendStringResponse("s"+"\r\n");
				String vel = TCP_IP_Handler.getStringRequest();
				double velocity = Double.valueOf(vel);
				setJointRelVel(velocity);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("change_obs_time")){
				System.out.println("Waiting for Time Parameter in change obs time");
				TCP_IP_Handler.sendStringResponse("s"+"\r\n");
				String obs_time = TCP_IP_Handler.getStringRequest();
				double new_time = Double.valueOf(obs_time);
				this.time_to_collect_obs_sec = new_time;
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
				System.out.println("Received: "+ String.valueOf(new_time));
			}
			
			else if(command_msg.equals("change_pi_z_threshold")){
				System.out.println("Waiting for PI parameter Z value");
				TCP_IP_Handler.sendStringResponse("s"+"\r\n");
				String z_val = TCP_IP_Handler.getStringRequest();
				double new_z_val = Double.valueOf(z_val);
				this.pi_z_threshold = new_z_val;
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("update_tool")){
				System.out.println("Waiting for Tool Params");
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("get_fk")){
				System.out.println("Sending Current Cartesian Position");
				Frame Current_Cart = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange());
				double[] cartesian_pose = new double[6];
				cartesian_pose[0] = Current_Cart.getX()/1000;
				cartesian_pose[1] = Current_Cart.getY()/1000;
				cartesian_pose[2] = Current_Cart.getZ()/1000;
				cartesian_pose[3] = Current_Cart.getAlphaRad();
				cartesian_pose[4] = Current_Cart.getBetaRad();
				cartesian_pose[5] = Current_Cart.getGammaRad();
//				System.out.println("Sending following pose:");
//				ReadWriteUtilities.display2DListDouble(cartesian_pose);
				ArrayList<double[]> current_cart_pose = new ArrayList<double[]>();
				current_cart_pose.add(cartesian_pose);
				TCP_IP_Handler.sendNPArrResponse(current_cart_pose);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("move_ptp")){
				System.out.println("Moving Robot in PTP Motion");
				ArrayList<double[]> cart_pose = TCP_IP_Handler.getDoubleNPArrRequest();
//				System.out.println("Received Following Pose");
//				ReadWriteUtilities.display2DListDouble(cart_pose);
				this.moveRobot(cart_pose, false);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
				
			}
			
			else if(command_msg.equals("move_lin")){
				System.out.println("Moving Robot in Linea Motion");
				ArrayList<double[]> cart_pose = TCP_IP_Handler.getDoubleNPArrRequest();
				System.out.println("Received Following Pose");
				ReadWriteUtilities.display2DListDouble(cart_pose);
				this.moveRobot(cart_pose, true);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
				
			}
			
			else if(command_msg.equals("send_joint_pos")){
				System.out.println("Sending Current Joint Position");
				JointPosition joint_position = lBR_iiwa_14_R820_1.getCurrentJointPosition();
				current_joint_position.clear();
				current_joint_position.add(joint_position.get());
				TCP_IP_Handler.sendNPArrResponse(current_joint_position);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("move_to_joint_pose")){
				System.out.println("Moving Robot in Linea Motion");
				ArrayList<double[]> joint_pose = TCP_IP_Handler.getDoubleNPArrRequest();
				System.out.println("Received Following Pose");
				ReadWriteUtilities.display2DListDouble(joint_pose);
				this.move_to_joint_pose(joint_pose);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			else if(command_msg.equals("recv_obs")){
				System.out.println("Sending observations");
//				ReadWriteUtilities.display2DListDouble(observation_data);
				TCP_IP_Handler.sendNPArrResponse(observation_data);
				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
				
			}
			else{
				System.out.println("Received Something Unusual, Closing Connection");
//				break;
//				TCP_IP_Handler.sendStringResponse("Done"+"\r\n");
			}
			
			
			
			
			
		}
		
		TCP_IP_Handler.closeConenction();
		
//		System.out.println("Moving the Robot to P2");
//		try{
//			peg_tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.1));
//		}
//		catch (Exception e) {
//			// TODO: handle exception
//			System.out.println("Could not move to P2 due to");
//			System.out.println(e.getMessage()); //
//		}
//		
//		
//        //Move Function
//        
//        ArrayList<String> FrameNames = new ArrayList<String> ();
//        FrameNames.add("/P2");
//        FrameNames.add("/P3");
//        FrameNames.add("/P4");
//        FrameNames.add("/P5");
//        FrameNames.add("/P6");
//        
//        final DataRecorder recorder = initDataRecorder();
//        recorder.startRecording();
//        
//        System.out.println("Executing Free Space");
//        ArrayList<double[]> cartesian_config = this.CreateCartesianTraj(FrameNames);
//        this.executeSplineMotionFromCartesianConfig(cartesian_config);
//        
//        FrameNames.clear();
//        
//        FrameNames.add("/P6");
//        FrameNames.add("/P7");
//        cartesian_config.clear();
//        
//        cartesian_config = this.CreateCartesianTraj(FrameNames);
//        System.out.println("Executing Insertion");
//        Spline myspline = new Spline(lin(getApplicationData().getFrame("/P6")),
//        							lin(getApplicationData().getFrame("/P7")));
//        
//        
//        
//        try {
//			TimeUnit.SECONDS.sleep(2);
//		} catch (InterruptedException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		}
//        ForceCondition Cond_1 = ForceCondition.createNormalForceCondition(peg_tcp, CoordinateAxis.Z, 30.0);
//        try{
//			peg_tcp.move(myspline.setJointVelocityRel(0.05).setMode(impedanceControlmode).breakWhen(Cond_1));
//		}
//		catch (Exception e) {
//			// TODO: handle exception
//			System.out.println("Could not move to P2 due to");
//			System.out.println(e.getMessage()); //
//		}
//        recorder.stopRecording();
        
	}
}