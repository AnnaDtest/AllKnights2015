package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.Servo; 

public class TPAServo extends Servo {
	
	// This is an object declaration that sets a named location in memory.
	private final TPAJoystick joystick;
	
	// define class variables for training board tests
	// no hard coding in the methods
	// private int button = 1;
	
    boolean deployState = false;
    boolean boltState = false;
	
	private double rotate1 = 0.5;
	private double rotate2 = 0.0; // originally 0
	
	// this is the required base class constructor. It references the base Servo class.
	// The channel parameter takes an integer argument. The channel refers to the PWM port
	// on which the servo is seated.
	// Final can only be initialized once. It does not need to be initialized at point of declaration
	public  TPAServo (final int channel, TPAJoystick joystick ) {
		
		// Super refers to parent instance variable in superclass.
		super(channel);
		// This joystick local to the class
		this.joystick=joystick;
	}
	
	// 
	public void runBolt() {
		
		
		// getButtonPush to execute with one button push
		if (joystick.getButtonPush(TPARobotMap.servoButton) == true ) {
			
			deployState = !deployState;
			if (deployState == true) {
				boltState = true;
				set(rotate1);
			}
			
			if (deployState == false) {
				boltState = false;
				set(rotate2);
			}
			
			
			
		} 
	}
	
	

}
