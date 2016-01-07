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

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//main class body
public class TPARobotDrive extends RobotDrive {
	private double m_magnitude;
	private double m_direction;
	private double m_rotation;
	
	// This is an object declaration that sets a named location in memory. It is a joystick 
	// object of type TPAJoystick. Or a reference variable of type TPAJoystick. 
	// "final" tells the compiler that subclass cannot override.
	private final TPAJoystick joystick ;
	
	// Constructor
	public TPARobotDrive(int frontLeftMotor, int rearLeftMotor, int frontRightMotor, int rearRightMotor, TPAJoystick joystick) {
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		this.joystick=joystick;
	}
	
	// Method
	public void mecanumDrive_Polar() {
		double throttle=(joystick.getRawAxis(4) - 1) / -2;
		// TPALCD.getInstance().println(1, "Speed mult: x" + throttle);
		SmartDashboard.putNumber("Speed multiplier", throttle);
		m_magnitude = joystick.getMagnitude() * throttle;
		m_direction = joystick.getDirectionDegrees();
		m_rotation = joystick.getTwist() * throttle;
		
		mecanumDrive_Polar(m_magnitude, m_direction, m_rotation);
	}
	
	// function
	public SpeedController getFrontLeftMotor() 
	{
		return m_frontLeftMotor;
	}
	
	// function 
	public SpeedController getFrontRightMotor()
	{
		return m_frontRightMotor;
	}
	
	
	public SpeedController getRearLeftMotor()
	{
		return m_rearLeftMotor;
	}
	
	public SpeedController getRearRightMotor()
	{
		return m_rearRightMotor;
	}

}
