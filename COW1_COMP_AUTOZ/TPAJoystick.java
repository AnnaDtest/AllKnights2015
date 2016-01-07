package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TPAJoystick extends Joystick{
	
	private boolean[] buttonState = new boolean[12];
    
    public TPAJoystick(final int aPort) {
        super(aPort);
        for(int i=0; i<buttonState.length; i++) {
            buttonState[i] = true;
        }
    }
    
    public boolean getButtonPush(int button) {
        boolean returnValue;
        // Call getRawButton from the Joystick class.
        boolean newState = super.getRawButton(button);
        if(newState == true && newState != buttonState[button - 1]) {
                returnValue = true;
        }
        else {returnValue = false;}
        buttonState[button - 1] = newState;
        return(returnValue);
    }
}
