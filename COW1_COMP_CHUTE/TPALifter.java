/**
 *
 *  @author - jd
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
import edu.wpi.first.wpilibj.Talon; 
import edu.wpi.first.wpilibj.DigitalInput;


//main class body
public class TPALifter {
	
	// Variable Declarations Here
	
	// Method booleans to control iterative processing, "static" means share with all methods in this class 
	private static boolean b_downlift, b_uplift, b_downManual, b_upManual, b_toteLowering, b_quickRaise;
	

	// Stops the motor, used in all methods across the entire class
	private static double stopMotor = 0 ;
	
	// method name in variable name
	double ServoRotateAngle     =  0.5 ;
	double QuickRaiseSpeed      = -0.5 ;
	double upLiftHighSpeed 		= -0.8 ; 	// negative current sends the lifter up
	double upLiftSlowSpeed 		= -0.2 ;
	double downLiftHighSpeed 	=  0.8 ;	// positive current sends the lifter down
	double downLiftSlowSpeed 	=  0.6 ;
	double downLiftCounter		=  0.1 ;
	double toteLoweringSpeed 	=  0.2 ;
	double toteLoweringCounter	=  0.0 ;
	double manualUpLiftSpeed 	= -0.6 ;
	double manualDownLiftSpeed 	=  0.6 ;

	
	// Object declaration that sets a named location in memory.
	TPAJoystick joystick;
	TPAServo servo;
	Talon talon;
	DigitalInput DigitalInput1, DigitalInput2, DigitalInput3, DigitalInput4, DigitalInput6;  // magnet breaks circuit and returns false, otherwise always returns true
	
	// Constructor Definition, takes joystick, servo, talon, and 4 digital input ports as args
	public TPALifter(TPAJoystick joystick,
					 TPAServo servo,
					 Talon talon, 
			     	 DigitalInput DigitalInput1,
			     	 DigitalInput DigitalInput2,
			     	 DigitalInput DigitalInput3,
			     	 DigitalInput DigitalInput4,
			     	 DigitalInput DigitalInput6){
	this.joystick = joystick;
	this.servo = servo;
	this.talon = talon;
	this.DigitalInput1 = DigitalInput1;
	this.DigitalInput2 = DigitalInput2;
	this.DigitalInput3 = DigitalInput3;
	this.DigitalInput4 = DigitalInput4;
	this.DigitalInput6 = DigitalInput6;
	}
	
	// Intended for instrumentation to be used at a later time
	public boolean getPortState(){
		return DigitalInput1.get(); //fix this later with array
	}
	
    
	// Automatically lifts lifter mechanism, starts high speed, then slows, then stops 
	// Operation based on limit switch values 
	
	public void upLift(){
		    
		//set lifter speed and boolean states for this method
		if(joystick.getButtonPush(TPARobotMap.UpLiftButton) && DigitalInput4.get() == true && DigitalInput6.get() == false){
			
			b_uplift 		= true;
			b_downlift 		= false;
			b_upManual 		= false;
			b_downManual 	= false;
			b_toteLowering  = false;
			b_quickRaise    = false;
			
			talon.set(upLiftHighSpeed); 
		} 
		
		    // b_uplift is true during upLift call but will be set false within the downLift method when downLift method is called
			if(DigitalInput3.get() 	== false   //Hit limit switch 3, begin slowing
				&& b_uplift  		== true
				&& b_downlift 		== false
				&& b_upManual 		== false 
				&& b_downManual 	== false 
				&& b_toteLowering  	== false
				&& b_quickRaise     == false) { 
				
				talon.set(upLiftSlowSpeed); 
			}
	
			if(DigitalInput4.get() 	== false   // Hit limit switch 4, stops lifter motor
				&& b_uplift 		== true
				&& b_downlift 		== false
				&& b_upManual 		== false
				&& b_downManual 	== false
				&& b_toteLowering  	== false
				&& b_quickRaise     == false){ 
				
				talon.set(stopMotor);
				b_uplift 		= false;  // set to false to reset boolean state eliminates iterative conflict 
				b_downlift 		= false;
			    b_upManual 		= false;
				b_downManual 	= false;
				b_toteLowering  = false;
				b_quickRaise    = false;
			}
	}
	
	// Automatically lowers lifter mechanism, starts high speed, then slows, then stops 
	// Operation based on limit switch values 
	public void downLift(){
		if(joystick.getButtonPush(TPARobotMap.DownLiftButton) && DigitalInput1.get() == true && servo.get() == 0 && DigitalInput6.get() == false){
		
			b_uplift 		= false;
			b_downlift 		= true;
			b_upManual 		= false;
			b_downManual 	= false;
			b_toteLowering  = false;
			b_quickRaise    = false;
			
			talon.set(downLiftHighSpeed); 
		}
		
		    // b_downlift is true during downLift call but will be set to false within the upLift method when upLift method is called
			if(DigitalInput2.get() 	== false // Hit limit switch 2, begin slowing
				&& b_downlift   	== true
				&& b_uplift     	== false
				&& b_upManual   	== false
				&& b_downManual 	== false 
				&& b_toteLowering  	== false
				&& b_quickRaise     == false){  
				
				talon.set(downLiftSlowSpeed);
			}
			
			if(DigitalInput1.get() 	== false // Hit limit switch 1, stops lifter motor
				&& b_downlift   	== true
				&& b_uplift     	== false
				&& b_upManual   	== false
				&& b_downManual 	== false 
				&& b_toteLowering  	== false
				&& b_quickRaise     == false){ 
				
				talon.set(downLiftCounter);
				
				b_uplift 		= false;  // set to false to reset boolean state eliminates iterative conflict 
				b_downlift 		= false;
			    b_upManual 		= false;
				b_downManual 	= false;
				b_toteLowering  = false;
				b_quickRaise    = false;
			}
		}
	
	// Lowers the lifter with aLL the six totes at slowest speed, ignores all switches but switch 2
	public void toteLowering(){ 
		if(joystick.getButtonPush(TPARobotMap.LoweringToteButton) && DigitalInput6.get() == false){
			
			b_uplift 		= false;
			b_downlift 		= false;
			b_downManual 	= false;
			b_upManual 		= false;
			b_toteLowering  = true;
			b_quickRaise    = false;
			
			talon.set(toteLoweringSpeed);
		}
		
		// Stop the lifter motor at switch 2
		if(DigitalInput2.get() 	== false 
			&& b_uplift     	== false
			&& b_downlift   	== false 
			&& b_downManual 	== false
			&& b_upManual   	== false
			&& b_toteLowering 	== true
			&& b_quickRaise     == false){
			
			talon.set(stopMotor);	
			b_uplift 		= false; // set to false to reset boolean state eliminates iterative conflict
			b_downlift 		= false;
			b_downManual 	= false;
			b_upManual 		= false;
			b_toteLowering  = false;
			b_quickRaise    = false;
		}
	}
	
	/*  Manually raises the lifter 
	 *  If button pushed, it sets the method booleans to false disabling the other methods 
	 *  so they will not interfere with manual lift operation.
	 *  b_upManual and Limit Switch 4 have to be true
	 *  Limit switches are true when they are not activated 
	 */
	public void manualUplift(){
		if(joystick.getRawButton(TPARobotMap.ManualUpLiftButton) == true && DigitalInput4.get() == true && servo.get() == 0 && DigitalInput6.get() == false){
			b_uplift 		= false;
			b_downlift 		= false;
			b_upManual 		= true;
		    b_downManual 	= false;
		    b_toteLowering  = false;
		    b_quickRaise    = false;
		    
		    // stops when it gets to limit switch 4
		    while(((joystick.getRawButton(TPARobotMap.ManualUpLiftButton) == true) && (DigitalInput4.get() == true))
		    	  && b_uplift 				== false
		    	  && b_downlift 			== false
		    	  && b_upManual 			== true
		    	  && b_downManual 			== false
		    	  && b_toteLowering 		== false
		    	  && b_quickRaise           == false){
		    	
		    	  talon.set(manualUpLiftSpeed);
		    	  // may need to remove || or logic 
		    	  if(((joystick.getRawButton(TPARobotMap.ManualUpLiftButton) == false) || (DigitalInput4.get() == false)) && b_upManual == true && b_downManual == false){ 
						
						talon.set(stopMotor);
						b_uplift 		= false;  // set to false to reset boolean state eliminates iterative conflict 
						b_downlift 		= false;
					    b_upManual 		= false;
						b_downManual 	= false;
						b_toteLowering  = false;
						b_quickRaise    = false;
				  } 
		    }
		}      
	}
	
	
	/*  Manually lowers the lifter 
	 *  If button pushed, it sets the method booleans to false disabling the other methods 
	 *  so they will not interfere with manual lift operation.
	 *  b_downManual and Limit Switch 1 have to be true
	 *  Limit switches are true when they are not activated 
	 */
	public void manualDownlift(){
		if(joystick.getRawButton(TPARobotMap.ManualDownLiftButton) && DigitalInput1.get() == true && servo.get() == 0 && DigitalInput6.get() == false){
			b_uplift 		= false;
			b_downlift 		= false;
			b_upManual 		= false;
			b_downManual 	= true;
			b_toteLowering  = false;
			b_quickRaise    = false;
			
			// stops when it gets to limit switch 1
			while(((joystick.getRawButton(TPARobotMap.ManualDownLiftButton) == true) && (DigitalInput1.get() == true))
				  && b_uplift 				== false
			      && b_downlift 			== false
			      && b_upManual 			== false
			      && b_downManual 			== true
			      && b_toteLowering 		== false
			      && b_quickRaise           == false){
				
				  talon.set(manualDownLiftSpeed);
				  
				  // Stop the talon motor
				  if(((joystick.getRawButton(TPARobotMap.ManualDownLiftButton) == false) || (DigitalInput1.get() == false)) && b_upManual == false && b_downManual == true){ 
						
						talon.set(stopMotor);
						b_uplift 		= false;  // set to false to reset boolean state eliminates iterative conflict 
						b_downlift 		= false;
					    b_upManual 		= false;
						b_downManual 	= false;
						b_toteLowering  = false;
						b_quickRaise    = false;
				  }
			}
		}

	}
	
	public void QuickRaise(){
		if(joystick.getButtonPush(TPARobotMap.QuickRaiseButton) && DigitalInput4.get() == true && DigitalInput6.get() == false && servo.get() == 0 ){
			b_uplift 		= false;
			b_downlift 		= false;
			b_upManual 		= false;
			b_downManual 	= false;
			b_toteLowering  = false;
			b_quickRaise    = true;
			
			servo.set(ServoRotateAngle);
			talon.set(QuickRaiseSpeed);
		}
		
		if(DigitalInput2.get() == false
		   && b_uplift         == false
		   && b_downlift 	   == false
		   && b_upManual 	   == false
		   && b_downManual 	   == false
		   && b_toteLowering   == false
		   && b_quickRaise     == true){
			
			talon.set(0);
			b_uplift 		= false;  // set to false to reset boolean state eliminates iterative conflict 
			b_downlift 		= false;
		    b_upManual 		= false;
			b_downManual 	= false;
			b_toteLowering  = false;
			b_quickRaise    = false;
		}
		
	}

// end class	
}
	
