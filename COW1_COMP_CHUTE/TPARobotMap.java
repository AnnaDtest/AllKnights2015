package org.usfirst.frc.team3944.robot;

public class TPARobotMap {

		//TPA Joystick
		public static final int JoystickPort = 0;
		 
		// PWM Ports 
		 // Wheel Talon Ports
		public static final int frontLeftPort 	= 0;
		public static final int rearLeftPort 	= 1;
		public static final int frontRightPort 	= 2;
		public static final int rearRightPort 	= 3;
		 // Lifter Talon Ports
		public static final int LiftTalonPort = 4;
		public static final int PushTalonPort = 5; 
		 // Servo Port
		public static final int servoPort = 9;  
		  
		
		// DIO Permanent Port Assignments 
		  // Limit Switch 
		public static final int LimitSwitch1 = 0;
		public static final int LimitSwitch2 = 1;
		public static final int LimitSwitch3 = 2;
		public static final int LimitSwitch4 = 3;
		public static final int LimitSwitch5 = 4;
		public static final int LimitSwitch6 = 5;
		public static final int positionSwitchRed   = 6;
		public static final int PositionSwitchGreen = 7;
		
		// Relay Port Assignments 
		public static final int SpikeRelay1 = 0;
		
		// Permanent Button Assignment
		
		public static final int QuickRaiseButton       = 1;
		public static final int servoButton            = 2;
		public static final int ManualPusherButton     = 0; 
		public static final int ManualRetracterButton  = 0; 
		public static final int ManualDownLiftButton   = 0;
		public static final int ManualUpLiftButton     = 0;
		public static final int LoweringToteButton     = 7; // origianlly 1
		public static final int PushandRetractButton   = 8;
		public static final int RetracterButton 	   = 9;
		public static final int PusherButton 	       = 10;
		public static final int DownLiftButton 	       = 11;
		public static final int UpLiftButton 	       = 12;
		
		// CAN ENCODERS 
		
		// CAN Port Assignments
		public static final int compressorCANPort = 0;
		public static final int airOutDoubleSolenoidCANPort = 0;
		public static final int airInDoubleSolenoidCANPort = 1;
		
		

		
	
}
