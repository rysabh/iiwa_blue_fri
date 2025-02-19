package utilities;

import com.kuka.roboticsAPI.deviceModel.*;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;


import utilities.Timedelay;

/**
 * Program to generate the plastic motion of the robot.
 * Plastic Motion : robot will be under compliance. When affected by the forces and changed its position and/or orientation, 
 * robot DOES NOT go to the original position.   
 */

public class PlasticMotionThread implements Runnable {
		private boolean exit; 
	    private String name; 
	    private LBR lBR_iiwa_7_R800_1;
	    private ObjectFrame tcp;
	    Thread t; 
	    IMotionContainer stablePos;
	  
	    public PlasticMotionThread(String name, LBR lBR_iiwa_7_R800_1, ObjectFrame tcp) 
	    { 
	    	this.lBR_iiwa_7_R800_1 = lBR_iiwa_7_R800_1;
	    	this.tcp = tcp;
	        this.name = name; 
	        t = new Thread(this, this.name); 
	        exit = false; 
	        t.start(); // Starting the thread 
	    } 
	
    public void run() {
        while(!exit)
    	{
			CartesianImpedanceControlMode mode;
			mode = 	new CartesianImpedanceControlMode();
			mode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(1000);
			mode.parametrize(CartDOF.Z).setStiffness(1000);
//			mode.parametrize(CartDOF.ROT).setStiffness(300);
			mode.parametrize(CartDOF.ALL).setDamping(1);
			stablePos = tcp.moveAsync((new PositionHold(mode, -1, null)));
			Frame F_in = lBR_iiwa_7_R800_1.getCurrentCartesianPosition(tcp);
			double x_in = F_in.getX();
			double y_in = F_in.getY();
			double z_in = F_in.getZ();
			
			while (!stablePos.isFinished() && !exit)
			{
				Frame F = lBR_iiwa_7_R800_1.getCurrentCartesianPosition(tcp);
				double x = F.getX();
				double y = F.getY();
				double z = F.getZ();
				double dist_val = Math.sqrt(Math.pow((x_in-x),2) + Math.pow((y_in-y),2)+ Math.pow((z_in-z),2));			
				if (dist_val > 5)
				{
					stablePos.cancel();
				}
				Timedelay.wait_milliseconds(2);
			}
		}
		if (!stablePos.isFinished())
		{
			stablePos.cancel();
		}
		
    }
    
    public void stop() 
    { 
        exit = true; 
    } 

}
