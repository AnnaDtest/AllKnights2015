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
import edu.wpi.first.wpilibj.Timer;
 
public class Robot extends IterativeRobot {
	/* Autonomous Variable Section 
	 * 
	 * Lifter Variables 
	 * Positive is down, Negative is up
	 */
	 private double autoInitialDownSpeed 	=  0.8;
	 private double autoInitialUpSpeed 		= -0.3;
	 private double autoSpeedStop			=  0;
	 
	 // Autonomous Orchestration Variables 
	 
	 // Stop Mecanum Variable for all orchestrations left, right, center
	 private double stop_mecanum_Mag 	=  0.0;
	 private double stop_mecanum_Dir	=  0.0;
	 private double stop_mecanum_Rot 	=  0.0;
	 
	 // Orchestration A Left Variables 
	 // Magnitude (speed), Direction (degrees), Rotation (Turning) 
	 private double orch_A_Left_Time 	=  3.0;
	 private double orch_A_Left_Mag 	= -0.4;
	 private double orch_A_Left_Dir	 	=  0.0;
	 private double orch_A_Left_Rot 	=  0.0;
	 
	 //  Orchestration B  Left Variables 
	 private double orch_B_Left_Time 	=  0.0;
	 private double orch_B_Left_Mag 	=  0.0;
	 private double orch_B_Left_Dir	 	=  0.0;
	 private double orch_B_Left_Rot 	=  0.0;
	  
	 // Orchestration A Right Variables
     // Magnitude (speed), Direction (degrees), Rotation (Turning) 
	 private double orch_A_Right_Time 	=  3.5;
	 private double orch_A_Right_Mag 	= -0.4;
	 private double orch_A_Right_Dir	=  0.0;
	 private double orch_A_Right_Rot 	=  0.0;
	
	// Orchestration B Right Variables
	 private double orch_B_Right_Time 	=  0.0;
	 private double orch_B_Right_Mag 	=  0.0;
	 private double orch_B_Right_Dir	=  0.0;
	 private double orch_B_Right_Rot 	=  0.0;
	 
	 // Orchestration A Center Variables 
     // Magnitude (speed), Direction (degrees), Rotation (Turning) 
	 private double orch_A_Center_Time 	=  3.75;
	 private double orch_A_Center_Mag 	= -0.4;
	 private double orch_A_Center_Dir	=  0.0;
	 private double orch_A_Center_Rot 	=  0.0;
	 
	 // Orchestration B Center Variables 
	 private double orch_B_Center_Time 	=  0.0;
	 private double orch_B_Center_Mag 	=  0.0;
	 private double orch_B_Center_Dir	=  0.0;
	 private double orch_B_Center_Rot 	=  0.0;
	 // End Orchestration Variables 
	 
	 //Autonomous Booleans
	 private boolean b_leftAutonomous, b_rightAutonomous, b_centerAutonomous, b_upAutonomous, b_downAutonomous; 
	 private boolean b_leftDriveAutonomous, b_rightDriveAutonomous, b_centerDriveAutonomous; 
	 private boolean controlChuteLeft[], controlChuteRight[], controlChuteCenter[];

	 //Autonomous Timer Variables 
	 private double MainTimer, autoTimerStart1Left, autoTimerStart2Left; 
	 private double autoTimerStart1Right, autoTimerStart2Right;
	 private double autoTimerStart1Center, autoTimerStart2Center;
	
	private TPAJoystick joystick;
	private TPAServo servo;
    private TPARobotDrive robotDrive;
	private TPADriveTrainTester driveTrainTester;
	private Timer timerMain;
	private Timer timer1Left;
	private Timer timer2Left;
	 
	private Timer timer1Right;
	private Timer timer2Right;

	private Timer timer1Center;
	private Timer timer2Center;
	
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
    	// Timer section 
    	timerMain = new Timer();
    	timerMain.reset();
    	timerMain.start();
    	
    	timer1Left = new Timer();
    	timer2Left = new Timer();
    	
    	timer1Right = new Timer();
    	timer2Right = new Timer();
    	
    	timer1Center = new Timer();
    	timer2Center = new Timer();
    	
    	controlChuteLeft   = new boolean[] {false, false};
    	controlChuteRight  = new boolean[] {false, false};
    	controlChuteCenter = new boolean[] {false, false};
    	controlChuteLeft[0] = true;
    	controlChuteRight[0] = true;
    	controlChuteCenter[0] = true;
    	 
    	/* Boolean setting section 
    	 * Robot Aligns Left
    	 */
        if  (DigitalInput7.get() == false && DigitalInput8.get() == true ) {
        	b_leftAutonomous 		= true;
        	b_rightAutonomous 		= false; 
            b_centerAutonomous 		= false; 
            b_upAutonomous			= false;
            b_downAutonomous   		= true;
            b_leftDriveAutonomous  	= false;
        }
        // Robot Aligns Right 
        if  (DigitalInput7.get() == true && DigitalInput8.get() == false ) {
        	b_leftAutonomous 		= false;
        	b_rightAutonomous 		= true; 
            b_centerAutonomous 		= false; 
            b_upAutonomous			= false;
            b_downAutonomous   		= true;
            b_rightDriveAutonomous  = false;
        }
        // Robot Aligns Center
        if  (DigitalInput7.get() == true && DigitalInput8.get() == true ) {
        	b_leftAutonomous 		= false;
        	b_rightAutonomous 		= false; 
            b_centerAutonomous 		= true; 
            b_upAutonomous			= false;
            b_downAutonomous   		= true;
            b_centerDriveAutonomous = false;
        }
    	
    } // End autonomousInit 
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    	// Start Timer 
    	MainTimer = timerMain.get();
        
        // Robot Aligns Left  
        if (DigitalInput6.get() 		== false   // Ensure pusher is fully retracted
       	    && DigitalInput7.get()  	== false 
            && DigitalInput8.get() 		== true 
            && b_leftAutonomous 		== true 
            && b_rightAutonomous 		== false
            && b_centerAutonomous 		== false
            && b_upAutonomous			== false
            && b_downAutonomous			== true
            && b_leftDriveAutonomous   	== false) {
       	   
            LifterTalon.set(autoInitialDownSpeed); 
            b_leftAutonomous 		= true; 
            b_rightAutonomous 		= false;
            b_centerAutonomous 		= false;
            b_upAutonomous			= true;
            b_downAutonomous   		= false;
            b_leftDriveAutonomous  	= false;   
        }
           
        // Lifter Descends To LimitSwitch 1, lifter immediately ascends, reset booleans   	  
        if (DigitalInput1.get() 		== false 
            && b_leftAutonomous 		== true 
            && b_rightAutonomous 		== false
            && b_centerAutonomous 		== false
            && b_upAutonomous			== true
            && b_downAutonomous			== false
            && b_leftDriveAutonomous   	== false) {
          	
            LifterTalon.set(autoInitialUpSpeed);
          	b_leftAutonomous 		= true; 
            b_rightAutonomous 		= false;
            b_centerAutonomous 		= false;
            b_upAutonomous			= false;
            b_downAutonomous		= false;
            b_leftDriveAutonomous  	= false;          
        }
          	 
        // Lifter ascends to LimitSwitch 4, stops motor, reset booleans 
        if (DigitalInput4.get() 		== false 
        	&& b_leftAutonomous 		== true 
            && b_rightAutonomous 		== false
            && b_centerAutonomous 		== false
            && b_upAutonomous			== false
            && b_downAutonomous			== false
            && b_leftDriveAutonomous   	== false) { 
   	      
            LifterTalon.set(autoSpeedStop);
       
            b_leftAutonomous 		= false; 
            b_rightAutonomous 		= false;
            b_centerAutonomous 		= false;
            b_upAutonomous			= false;
            b_downAutonomous		= false;
            b_leftDriveAutonomous  	= true;
            // Delay one second
            Timer.delay(1);
            timer1Left.reset();
        	timer1Left.start();

   	  } 


         // Begin driving no limit switch since we might drive while lifting the can TBD. 
         if (b_leftDriveAutonomous 			== true
        	 && b_leftAutonomous 			== false 
             && b_rightAutonomous 			== false
        	 && b_centerAutonomous 			== false
             && b_upAutonomous				== false
             && b_downAutonomous			== false) {
       	  
        	 	 // Orchestration A Left
         		 if (controlChuteLeft[0] == true) {
         			 while ((autoTimerStart1Left = timer1Left.get()) < orch_A_Left_Time) { 
         			  robotDrive.mecanumDrive_Polar(orch_A_Left_Mag, orch_A_Left_Dir, orch_A_Left_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning) 
         			 }
         			
         			  controlChuteLeft[0] = false;
         			  controlChuteLeft[1] = true;
         	     
         			  timer2Left.reset();
         			  timer2Left.start(); 
         	    } 
        	 
       
         		//  Orchestration B  Left
        	    if (controlChuteLeft[1] == true) {	
        		    while ((autoTimerStart2Left = timer2Left.get()) < orch_B_Left_Time) { 
        		     robotDrive.mecanumDrive_Polar(orch_B_Left_Mag, orch_B_Left_Dir, orch_B_Left_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning)
        		    } 
        		 
          	         controlChuteLeft[0] = false;
          	         controlChuteLeft[1] = false; 
        	    }
   	 
        	    // Stop mecanumDrive 
        	    if (controlChuteLeft[0] 	== false
           		    && controlChuteLeft[1] 	== false){

        	    	robotDrive.mecanumDrive_Polar(stop_mecanum_Mag, stop_mecanum_Dir, stop_mecanum_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning)
              	     
           	  	 	b_leftDriveAutonomous   = false;
           	  	 	b_leftAutonomous 		= false; 
           	  	 	b_rightAutonomous 		= false;
           	  	 	b_centerAutonomous 		= false;
           	  	 	b_upAutonomous			= false;
           	  	 	b_downAutonomous		= false;
                }
     
        } 
         
          // Robot Aligns Right  
          if (DigitalInput6.get() 		== false   // pusher is fully retracted
              && DigitalInput7.get() 	== true 
              && DigitalInput8.get() 	== false 
              && b_leftAutonomous 		== false
              && b_rightAutonomous 		== true
              && b_centerAutonomous 	== false
              && b_upAutonomous			== false
              && b_downAutonomous		== true
              && b_rightDriveAutonomous == false) {
           	    	   
              LifterTalon.set(autoInitialDownSpeed); 
           
              b_leftAutonomous 			= false; 
              b_rightAutonomous 		= true;
              b_centerAutonomous 		= false;
              b_upAutonomous			= true;
              b_downAutonomous   		= false;
              b_rightDriveAutonomous    = false;
           }
           	        
          // Lifter Descends To LimitSwitch 1, lifter immediately ascends, reset booleans   
          if (DigitalInput1.get() 		== false 
              && b_leftAutonomous 		== false 
              && b_rightAutonomous 		== true
              && b_centerAutonomous 	== false
              && b_upAutonomous			== true
              && b_downAutonomous		== false
              && b_rightDriveAutonomous == false) {
           	       	
              LifterTalon.set(autoInitialUpSpeed);
              b_leftAutonomous 			= false; 
              b_rightAutonomous 		= true;
              b_centerAutonomous 		= false;
              b_upAutonomous			= false;
              b_downAutonomous			= false;
              b_rightDriveAutonomous	= false;
           	         
          }
           	       	 
          // Lifter ascends to LimitSwitch 4, stops motor, reset booleans 
          if(DigitalInput4.get() 		== false 
             && b_leftAutonomous 		== false 
             && b_rightAutonomous 		== true
             && b_centerAutonomous 		== false
             && b_upAutonomous			== false
             && b_downAutonomous		== false
             && b_rightDriveAutonomous  == false) { 
           		      
            LifterTalon.set(autoSpeedStop);
            b_leftAutonomous 			= false; 
            b_rightAutonomous 			= false;
            b_centerAutonomous 			= false;
            b_upAutonomous				= false;
            b_downAutonomous			= false;
            b_rightDriveAutonomous   	= true;
      
            // Delay one second
            Timer.delay(1);
            timer1Right.reset();
        	timer1Right.start();
           }
          
          
          // Begin driving no limit switch since we might drive while lifting the can TBD. 
          if (b_rightDriveAutonomous 	== true
              && b_leftAutonomous 		== false 
              && b_rightAutonomous 		== false
          	  && b_centerAutonomous 	== false
              && b_upAutonomous			== false
              && b_downAutonomous		== false) {
         	        	  
        	 // Orchestration A Right
      		 if (controlChuteRight[0] == true) {
      			 while ((autoTimerStart1Right = timer1Right.get()) < orch_A_Right_Time) { 
      			  robotDrive.mecanumDrive_Polar(orch_A_Right_Mag, orch_A_Right_Dir, orch_A_Right_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning) 
      			 }
      			
      			  controlChuteRight[0] = false;
      			  controlChuteRight[1] = true;
      	     
      			  timer2Right.reset();
      			  timer2Right.start(); 
      	     }
      		 
      		// Orchestration B  Right
     	    if (controlChuteRight[1] == true) {	
     		    while ((autoTimerStart2Right = timer2Right.get()) < orch_B_Right_Time) { 
     		     robotDrive.mecanumDrive_Polar(orch_B_Right_Mag, orch_B_Right_Dir, orch_B_Right_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning)
     		    } 
     		 		 
       	         controlChuteRight[0] = false;
       	         controlChuteRight[1] = false;  
     	    }
      		   	   
    	    // Stop mecanumDrive 
    	    if (controlChuteRight[0] 	== false
       		    && controlChuteRight[1] == false){
    	    	
    	    	robotDrive.mecanumDrive_Polar(stop_mecanum_Mag, stop_mecanum_Dir, stop_mecanum_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning)
          	    
    	    	b_rightDriveAutonomous   = false;
    	    	b_leftAutonomous 		 = false; 
    	    	b_rightAutonomous 		 = false;
    	    	b_centerAutonomous 	 	 = false;
    	    	b_upAutonomous			 = false;
    	    	b_downAutonomous		 = false;
    	    }
    	    
          } // End Robot Aligns Right
        
     
           // Robot Aligns Center 
           if (DigitalInput6.get() 			== false   // pusher is fully retracted
           	   && DigitalInput7.get() 		== true 
           	   && DigitalInput8.get() 		== true
           	   && b_leftAutonomous 			== false
           	   && b_rightAutonomous 		== false
           	   && b_centerAutonomous 		== true
           	   && b_upAutonomous			== false
           	   && b_downAutonomous			== true
           	   && b_centerDriveAutonomous   == false) {
           	        	    	   
           		LifterTalon.set(autoInitialDownSpeed); 
           	   b_leftAutonomous 		= false; 
           	   b_rightAutonomous 		= false;
           	   b_centerAutonomous 		= true;
           	   b_upAutonomous			= true;
           	   b_downAutonomous    		= false;
           	   b_centerDriveAutonomous  = false;         	        	  
           }
           	        	        
           // Lifter Descends To LimitSwitch 1, lifter immediately ascends, reset booleans   	         	        	         
           if (DigitalInput1.get() 			== false 
        	   && b_leftAutonomous 			== false 
           	   && b_rightAutonomous 		== false
           	   && b_centerAutonomous 		== true
           	   && b_upAutonomous			== true
           	   && b_downAutonomous			== false
           	   && b_centerDriveAutonomous   == false) {
           	        	       	
           		LifterTalon.set(autoInitialUpSpeed);
           	   b_leftAutonomous 		= false; 
           	   b_rightAutonomous 		= false;
           	   b_centerAutonomous 		= true;
           	   b_upAutonomous			= false;
           	   b_downAutonomous			= false;
           	   b_centerDriveAutonomous	= false;         	        	         
           }
           	        	       	 
           // Lifter ascends to LimitSwitch 4, stops motor, reset booleans 
           if(DigitalInput4.get() 			== false 
              && b_leftAutonomous 			== false 
              && b_rightAutonomous 			== false
              && b_centerAutonomous 		== true
              && b_upAutonomous				== false
              && b_downAutonomous			== false
              && b_centerDriveAutonomous    == false) { 
           	        		      
              LifterTalon.set(autoSpeedStop);
              b_leftAutonomous 			= false; 
              b_rightAutonomous 		= false;
              b_centerAutonomous 		= false;
              b_upAutonomous			= false;
              b_downAutonomous			= false;
              b_centerDriveAutonomous  	= true;
              
              // Delay one second
              Timer.delay(1);
              timer1Center.reset();
          	  timer1Center.start();
            }
          
        // Begin driving no limit switch since we might drive while lifting the can TBD. 
           if (b_centerDriveAutonomous 		== true
        	   && b_leftAutonomous 			== false 
               && b_rightAutonomous 		== false
           	   && b_centerAutonomous 		== false
               && b_upAutonomous			== false
               && b_downAutonomous			== false) {
          	  
        	     // Orchestration A Center
        		 if (controlChuteCenter[0] == true) {
        			 while ((autoTimerStart1Center = timer1Center.get()) < orch_A_Center_Time) { 
        			  robotDrive.mecanumDrive_Polar(orch_A_Center_Mag, orch_A_Center_Dir, orch_A_Center_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning) 
        			 }
        			
        			  controlChuteCenter[0] = false; 
        			  controlChuteCenter[1] = true;
        			  timer2Center.reset();
                  	  timer2Center.start();
        	     }
        		 
        		// Orchestration B Center
          	    if (controlChuteCenter[1] == true) {	
          		    while ((autoTimerStart2Center = timer2Center.get()) < orch_B_Center_Time) { 
          		     robotDrive.mecanumDrive_Polar(orch_B_Center_Mag, orch_B_Center_Dir, orch_B_Center_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning)
          		    } 
          		 
            	         controlChuteCenter[0] = false;
            	         controlChuteCenter[1] = false;  
          	    }
        		 
         	    // Stop mecanumDrive Center
         	    if (controlChuteCenter[0] == false
         	    	&& controlChuteCenter[1] == false){ 
         	    	
         	    	robotDrive.mecanumDrive_Polar(stop_mecanum_Mag, stop_mecanum_Dir, stop_mecanum_Rot); // Magnitude (speed), Direction (degrees), Rotation (Turning)
               	    
         	    	b_centerDriveAutonomous  = false;
         	    	b_leftAutonomous 		 = false; 
         	    	b_rightAutonomous 		 = false;
         	    	b_centerAutonomous 	  	 = false;
         	    	b_upAutonomous			 = false;
         	    	b_downAutonomous		 = false;
         	    }
           } // End Robot Aligns Center 

          
           SmartDashboard.putBoolean("switch 1", DigitalInput1.get());
           SmartDashboard.putBoolean("switch 2", DigitalInput2.get());
           SmartDashboard.putBoolean("switch 3", DigitalInput3.get());
           SmartDashboard.putBoolean("switch 4", DigitalInput4.get());
           SmartDashboard.putBoolean("switch 5", DigitalInput5.get());
           SmartDashboard.putBoolean("switch 6", DigitalInput6.get());   
           SmartDashboard.putNumber("Lifting Talon Speed", LifterTalon.get());
           SmartDashboard.putNumber("Pushing Talon Speed", PushingTalon.get());
           SmartDashboard.putNumber("servo position zero means brake is on",  servo.get());
           SmartDashboard.putBoolean("Position switch Red", DigitalInput7.get());
           SmartDashboard.putBoolean("Position switch Green", DigitalInput8.get());
           SmartDashboard.putNumber("MainTimer", MainTimer);
           SmartDashboard.putNumber("autoTimerStart1Left", autoTimerStart1Left);
           SmartDashboard.putNumber("autoTimerStart2Left", autoTimerStart2Left);         
           SmartDashboard.putNumber("autoTimerStart1Right", autoTimerStart1Right);
           SmartDashboard.putNumber("autoTimerStart2Right", autoTimerStart2Right);        
           SmartDashboard.putNumber("autoTimerStart1Center", autoTimerStart1Center);
           SmartDashboard.putNumber("autoTimerStart2Center", autoTimerStart2Center);
           
           
    } //End autonomousPeriodic

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
