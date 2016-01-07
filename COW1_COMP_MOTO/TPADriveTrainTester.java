/**
 *
 *  @author - gdl
 *  @version  1.00 01/10/2015
 *  
 *  team #   -- 3944
 *  
 * COMMENTS:
 *
 * This is called by the main Robot class.
 *       
 *
 * REVISIONS:
 * 
 *  Deployment - 1.00 - jd  - Initial Deployment
 *
 */

// Imports go here
package org.usfirst.frc.team3944.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//main class body
public class TPADriveTrainTester {
	private TPAJoystick joystick;
	private TPARobotDrive robotDrive;
	
	// An array to track the state of each wheel
	private boolean motorStates[];
	private final int 
	    FRONT_LEFT 	= 0,
		REAR_LEFT 	= 1,
		FRONT_RIGHT = 2,
		REAR_RIGHT 	= 3;
	
	// Define the speeds for the ON and OFF states
	private final double 
	    ON = 0.75, 
		OFF = 0;
	
	public TPADriveTrainTester(TPAJoystick joystick, TPARobotDrive robotDrive) {
		this.joystick = joystick;
		this.robotDrive = robotDrive;
		motorStates = new boolean[4];
	}
	
	public void reset() {
		for (int i = 0; i < motorStates.length; i++) {
			motorStates[i] = false;
		}
	}
	
	public void run() {
		/**
		 * Looking at the top of the logitech 3D Pro:
		 * Top left button: 5
		 * Top right button: 6
		 * Bottom left button: 3
		 * Bottom right button: 4
		 */
		
		// the = ! means flip to the opposite value Need more info to understand 
		if (joystick.getButtonPush(5))
		{
			motorStates[FRONT_LEFT] = ! motorStates[FRONT_LEFT];
		}
		if (joystick.getButtonPush(6))
		{
			motorStates[FRONT_RIGHT] = ! motorStates[FRONT_RIGHT];
		}
		if (joystick.getButtonPush(3))
		{
			motorStates[REAR_LEFT] = ! motorStates[REAR_LEFT];
		}
		if (joystick.getButtonPush(4))
		{
			motorStates[REAR_RIGHT] = ! motorStates[REAR_RIGHT];
		}
			
		/**
		 * Set all the motors to on or off depending on the corresponding boolean value in motorStates
		 * TODO: rewrite the output class. LCD is no longer used. Options: SmartDashboard or log file.
		 * 
		 */
		
		// FRONT LEFT
		robotDrive.getFrontLeftMotor().set(
				motorStates[FRONT_LEFT] ? ON : OFF);
		//TPALCD.getInstance().println(3, "FL: " + 
		//		(motorStates[FRONT_LEFT] ? "ON" : "OFF"));
		SmartDashboard.putBoolean("Front Left Motor", motorStates[FRONT_LEFT]);
		// FRONT RIGHT 
		robotDrive.getFrontRightMotor().set(
				motorStates[FRONT_RIGHT] ? ON : OFF);
		//TPALCD.getInstance().println(4, "FR: " +
		//		(motorStates[FRONT_RIGHT] ? "ON" : "OFF"));
		SmartDashboard.putBoolean("Front Right Motor", motorStates[FRONT_RIGHT]);
		// REAR LEFT 
		robotDrive.getRearLeftMotor().set(  
				motorStates[REAR_LEFT] ? ON : OFF);
		//TPALCD.getInstance().println(5, "RL: " + 
		//		(motorStates[REAR_LEFT] ? "ON" : "OFF"));
		// REAR RIGHT
		SmartDashboard.putBoolean("Rear Left Motor", motorStates[REAR_LEFT]);
		robotDrive.getRearRightMotor().set(  
				motorStates[REAR_RIGHT] ? ON : OFF);
		//TPALCD.getInstance().println(6, "RR: " + 
		//		(motorStates[REAR_RIGHT] ? "ON" : "OFF"));
		SmartDashboard.putBoolean("Rear Right Motor", motorStates[REAR_RIGHT]);
			
	}
	

}
