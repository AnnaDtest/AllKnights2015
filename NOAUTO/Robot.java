/**
 *
 * 
 *  @version  1.00 01/10/2015
 *  
 *  team #   -- 3944
 *  
 * COMMENTS:
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *       
 *
 * REVISIONS:
 * 
 *  Deployment - 1.00 - jd  - Initial Deployment
 *
 */

// Imports go here
package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.livewindow.*;
import edu.wpi.first.wpilibj.Talon; 
import edu.wpi.first.wpilibj.DigitalInput;
 
 
public class Robot extends IterativeRobot {
 
	
	private TPAJoystick joystick;
	private TPAServo servo;
    private TPARobotDrive robotDrive;
	private TPADriveTrainTester driveTrainTester;
 
    Talon LifterTalon, PushingTalon;
    DigitalInput DigitalInput1, 
    			 DigitalInput2, 
    			 DigitalInput3, 
    			 DigitalInput4,
    			 DigitalInput5, 
    			 DigitalInput6,
    			 DigitalInput7, 
    			 DigitalInput8;
    
    private TPALifter lifter;
    private TPAPusher pusher;
    
    public void robotInit() {
    
    joystick = new TPAJoystick(TPARobotMap.JoystickPort);	
    robotDrive = new TPARobotDrive(	TPARobotMap.frontLeftPort,
    							 	TPARobotMap.rearLeftPort,
    								TPARobotMap.frontRightPort,
    								TPARobotMap.rearRightPort,
    								joystick);
    
    //Inverts the motor direction to support mecanum drive
	//setInvertedMotor(RobotDrive.MotorType motor, boolean isInverted)
    robotDrive.setInvertedMotor(TPARobotDrive.MotorType.kFrontRight, true);
    robotDrive.setInvertedMotor(TPARobotDrive.MotorType.kRearRight, true);
    
    driveTrainTester = new TPADriveTrainTester(joystick, robotDrive);
     
    servo = new TPAServo(TPARobotMap.servoPort, joystick);
    	
    // Tote manipulating object instantiations 	
    LifterTalon   = new Talon(TPARobotMap.LiftTalonPort);
    PushingTalon  = new Talon(TPARobotMap.PushTalonPort);
    DigitalInput1 = new DigitalInput(TPARobotMap.LimitSwitch1);
    DigitalInput2 = new DigitalInput(TPARobotMap.LimitSwitch2);
    DigitalInput3 = new DigitalInput(TPARobotMap.LimitSwitch3);
    DigitalInput4 = new DigitalInput(TPARobotMap.LimitSwitch4);
    DigitalInput5 = new DigitalInput(TPARobotMap.LimitSwitch5);
    DigitalInput6 = new DigitalInput(TPARobotMap.LimitSwitch6);
    DigitalInput7 = new DigitalInput(TPARobotMap.positionSwitchRed);
    DigitalInput8 = new DigitalInput(TPARobotMap.PositionSwitchGreen);
    
    
    lifter = new TPALifter(joystick, 
    		               servo,
    					   LifterTalon,
    					   DigitalInput1,
    					   DigitalInput2, 
    					   DigitalInput3, 
    					   DigitalInput4,
    					   DigitalInput6);
    
    pusher = new TPAPusher(joystick, 
    					   PushingTalon, 
    					   DigitalInput5, 
    					   DigitalInput6,
    					   servo);
    }
    public void autonomousInit(){
    	
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    }  // End AutonomousPeriodic
    
    /**
     * This function is called periodically during operator control
     */
   public void teleopPeriodic() {
    	
	    robotDrive.mecanumDrive_Polar();
        lifter.upLift();
        lifter.downLift();
        lifter.QuickRaise(); 
        lifter.toteLowering();// lowers 6 totes and a can at .1 speed
        //lifter.manualUplift();   
        //lifter.manualDownlift();
        servo.runBolt();
        pusher.PushandRetract(); // push one button to push and retract
        pusher.Push();           // full push with one button press
        pusher.Retract();        // full retract with one button press
        //pusher.ManualPush();
        //pusher.ManualRetract();
    
    	
        SmartDashboard.putBoolean("switch 1", DigitalInput1.get());
        SmartDashboard.putBoolean("switch 2", DigitalInput2.get());
        SmartDashboard.putBoolean("switch 3", DigitalInput3.get());
        SmartDashboard.putBoolean("switch 4", DigitalInput4.get());
        SmartDashboard.putBoolean("switch 5", DigitalInput5.get());
        SmartDashboard.putBoolean("switch 6", DigitalInput6.get());
        SmartDashboard.putNumber("Lifting Talon Speed", LifterTalon.get());
        SmartDashboard.putNumber("Pushing Talon Speed", PushingTalon.get());
        SmartDashboard.putNumber("servo position zero means brake is on",  servo.get());
       
        
        
        
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	
    	 driveTrainTester.run();
    	 SmartDashboard.putBoolean("switch 1", DigitalInput1.get());
         SmartDashboard.putBoolean("switch 2", DigitalInput2.get());
         SmartDashboard.putBoolean("switch 3", DigitalInput3.get());
         SmartDashboard.putBoolean("switch 4", DigitalInput4.get());
         SmartDashboard.putBoolean("switch 5", DigitalInput5.get());
         SmartDashboard.putBoolean("switch 6", DigitalInput6.get());
         SmartDashboard.putNumber("Lifting Talon Speed", LifterTalon.get());
         SmartDashboard.putNumber("Pushing Talon Speed", PushingTalon.get());
         
    
    }
    
}
