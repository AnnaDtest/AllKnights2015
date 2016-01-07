/**
 *
 *  @author - 
 *  @version  1.00 01/10/2015
 *  
 *  team #   -- 3944
 *  
 * COMMENTS:

	 * In robot.java UpLift and DownLift iterate every 20MS
	 * The b_uplift and b_downlift are sentinel variables that disable the 
	 * getButtonPush if statement. Button push sets the initial speed in
	 * both classes. However the limit switch logic remains open to each iteration
	 * of robot.java class. So when UpLift is running it sets b_uplift to true
	 * that activates the function of the limit switches germane to the uplift method
	 * It negates the boolean for b_downlift so the switch logic will fail for 
	 * DownLift. 
	 * 
 *
 *       
 *
 * REVISIONS:
 * 
 *  Deployment - 1.00 - jd  - Initial Deployment
 *  02/21/2015 - 1.01 - sd  - renamed booleans, tightened up logic 
 *
 */

// Imports go here
package org.usfirst.frc.team3944.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Talon; 
import edu.wpi.first.wpilibj.DigitalInput;


//main class body
public class TPAPusher {
	
	// Variable Declarations Here
	
	
	// Stops the motor, used in all methods across the entire class
		private static double stopMotor = 0 ;
		
		// method name in variable name
		double ServoRotateAngle     =  0.0 ; 
		double prExtendHighSpeed 	=  0.8 ; 	// positive current extends pusher
		double prRetractHighSpeed 	= -0.9 ; 	// Negative current sends the lifter down
		double prTimer 				=  1.0 ; 	// time in seconds
		double pushHighSpeed 		=  0.3 ;	
		double retractHighSpeed 	= -0.3 ;
		double manPushHighSpeed 	=  0.4 ;
		double manRetractHighSpeed 	= -0.4 ;
		
	
	// Method booleans to control iterative processing, "static" means share with all methods in this class
	private static boolean b_push, b_retract, b_manualPush, b_manualRetract, b_AutoPushRetract;
	
	
	
	
	// Object declaration that sets a named location in memory.
	TPAServo servo;
	Timer timer;
	TPAJoystick joystick;
	Talon talon;
	DigitalInput DigitalInput5, DigitalInput6;  // Pusher limit switches
	
	// Constructor Definition, takes joystick, talon, and 2 digital input ports as args
	public TPAPusher(TPAJoystick joystick, 
			         Talon talon,
					 DigitalInput DigitalInput5,
					 DigitalInput DigitalInput6,
					 TPAServo servo){
		
		this.joystick = joystick;
		this.talon = talon;
		this.DigitalInput5 = DigitalInput5;
		this.DigitalInput6 = DigitalInput6;
		this.servo = servo;
	}
	
	// Push and Retract with only one button Push, button 3
	public void PushandRetract(){ 
		
		// push at high speed
		if(joystick.getButtonPush(TPARobotMap.PushandRetractButton) && DigitalInput6.get() == false){
			b_push 				= false;
			b_retract 			= false;
			b_manualPush 		= false;
			b_manualRetract 	= false;
			b_AutoPushRetract 	= true;
			talon.set(prExtendHighSpeed);		// positive current	
		}
		
		// At limit switch 5 stop, wait 1 second, reverse motor, b_retract to true 
		if(DigitalInput5.get() 		== false 
		   && b_push 				== false 
		   && b_retract 			== false 
		   && b_manualPush 			== false 
		   && b_manualRetract 		== false 
		   && b_AutoPushRetract 	== true){
			talon.set(stopMotor);
			Timer.delay(prTimer);
			talon.set(prRetractHighSpeed);  // Negative current	
			b_retract = true;
		}
		
		// At limit switch 6 stop motor, set b_retract to false
		if(DigitalInput6.get() 		== false 
		   && b_push 				== false 
		   && b_retract 			== true 
		   && b_manualPush 			== false 
		   && b_manualRetract 		== false
		   && b_AutoPushRetract 	== true){
			
			talon.set(stopMotor);
			servo.set(ServoRotateAngle);
			b_push 				= false; // set to false to reset boolean state eliminates iterative conflict
			b_retract 			= false;
			b_manualPush 		= false;
			b_manualRetract 	= false;
			b_AutoPushRetract 	= false; 
		}
	}
	
	// Start push method, button 9
	public void Push(){
		if(joystick.getButtonPush(TPARobotMap.PusherButton) && DigitalInput5.get() == true && DigitalInput6.get() == false ){	
			b_push 				= true;
			b_retract 			= false;  
			b_manualPush 		= false;
			b_manualRetract 	= false;
		    b_AutoPushRetract 	= false;
			talon.set(pushHighSpeed);  // positive current
		}
		
		// At limit switch 5 stop motor, set b_push to false
		if(DigitalInput5.get() 	== false 
		   && b_push 			== true 
		   && b_retract 		== false 
		   && b_manualPush 		== false 
		   && b_manualRetract 	== false
		   && b_AutoPushRetract == false){
			
			talon.set(stopMotor);  
			b_push 				= false; // set to false to reset boolean state eliminates iterative conflict
			b_retract 			= false;
			b_manualPush 		= false;
			b_manualRetract 	= false;
			b_AutoPushRetract 	= false;
		}
	}
	
	// Start retract method, button 10
	public void Retract(){
		if(joystick.getButtonPush(TPARobotMap.RetracterButton) && DigitalInput5.get() == false && DigitalInput6.get() == true ){
			b_push 				= false;
			b_retract 			= true;
			b_manualPush 		= false;
			b_manualRetract 	= false;
			b_AutoPushRetract 	= false;
			talon.set(retractHighSpeed);  // negative current
			
		}
		// At limit switch 6 stop motor, set b_retract to false
		if(DigitalInput6.get() 	== false 
		   && b_push 			== false 
		   && b_retract 		== true 
		   && b_manualPush 		== false 
		   && b_manualRetract 	== false
		   && b_AutoPushRetract == false){
			
			talon.set(stopMotor);
			b_push 				= false; // set to false to reset boolean state eliminates iterative conflict
			b_retract 			= false;
			b_manualPush 		= false;
			b_manualRetract 	= false;
			b_AutoPushRetract 	= false;
		}
	
	}
	
	// manual push while holding down a button, button 5
	public void ManualPush(){
		if(joystick.getRawButton(TPARobotMap.ManualPusherButton) && DigitalInput5.get() == true){  
			b_push 				= false;
			b_retract 			= false;
			b_manualPush 		= true;
			b_manualRetract 	= false;
			b_AutoPushRetract 	= false;
			
			// runs while button and limit switch 5 return true
			while(joystick.getRawButton(TPARobotMap.ManualPusherButton) == true
				  && b_push				 == false
				  && b_retract			 == false
				  && b_manualPush 		 == true
				  && b_manualRetract	 == false
				  && b_AutoPushRetract 	 == false
				  && DigitalInput5.get() == true){
				
				  talon.set(manPushHighSpeed);  // positive current
				  
				  // may need to remove || or logic
				  if(((joystick.getRawButton(TPARobotMap.ManualPusherButton) == false) || (DigitalInput5.get() == false)) && b_manualPush == true && b_manualRetract == false){ 
						
						talon.set(stopMotor);
						b_push 				= false;
						b_retract 			= false;
						b_manualPush 		= false;
						b_manualRetract 	= false;
						b_AutoPushRetract 	= false;
				  } 
			}
		
	    }
	}
	
	// manual retract while holding down a button, button 6
	public void ManualRetract(){
		if(joystick.getRawButton(TPARobotMap.ManualRetracterButton) && DigitalInput6.get() == true){  
			
			b_push 				= false;
			b_retract 			= false;
			b_manualPush 		= false;
			b_manualRetract 	= true;
			b_AutoPushRetract 	= false;
			
			// runs while button and limit switch 6 return true
	        while(joystick.getRawButton(TPARobotMap.ManualRetracterButton) == true
	        	  && b_push 			 == false
	        	  && b_retract 			 == false
	        	  && b_manualPush 		 == false
	        	  && b_manualRetract 	 == true
	        	  && b_AutoPushRetract 	 == false
	        	  && DigitalInput6.get() == true){
	        	
	        	  talon.set(manRetractHighSpeed);
	        	  
	        	// may need to remove || or logic
				if(((joystick.getRawButton(TPARobotMap.ManualRetracterButton) == false) || (DigitalInput6.get() == false)) && b_manualPush == false && b_manualRetract == true){ 
						
						talon.set(stopMotor);
						b_push 				= false;
						b_retract 			= false;
						b_manualPush 		= false;
						b_manualRetract 	= false;
						b_AutoPushRetract 	= false;
				} 
	        }
		}
	}
	
// end class	
}
