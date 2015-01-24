// ****************************************************************************
//
// ExtremePro3DJoysticks.java
//
// Modified Nathan Finney, 2TrainRobotics 
// Original Source:
//     http://www.chiefdelphi.com/forums/archive/index.php/t-91420.html
//     Response by dbeckwith
//
// ****************************************************************************

package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Joystick;


public class ExtremePro3DJoystick extends Joystick {
    
    final static int NUM_AXES = 6;
    final static int NUM_BUTTONS = 12;
    
    public ExtremePro3DJoystick(int port) {

        super(port, NUM_AXES, NUM_BUTTONS);
        this.setAxisChannel(Joystick.AxisType.kX, 1);         // side to side
        this.setAxisChannel(Joystick.AxisType.kY, 2);         // forward/back
        this.setAxisChannel(Joystick.AxisType.kTwist, 3);     // twist
        
        this.setAxisChannel(Joystick.AxisType.kThrottle, 4);
        
    }
    
    
    
    public String toString(){
        
        String out =  "Button 1: " + this.getRawButton(1) + "\n"
                + "Button 2: " + this.getRawButton(2) + "\n"
                + "Button 3: " + this.getRawButton(3) + "\n"
                + "Button 4: " + this.getRawButton(4) + "\n"
                + "Button 5: " + this.getRawButton(5) + "\n"
                + "Button 6: " + this.getRawButton(6) + "\n"
                + "Button 7: " + this.getRawButton(7) + "\n"
                + "Button 8: " + this.getRawButton(8) + "\n"
                + "Button 9: " + this.getRawButton(9) + "n"
                + "Button 10: " + this.getRawButton(10) + "\n"
                + "Button 11: " + this.getRawButton(11) + "\n"
                + "Button 12: " + this.getRawButton(12) + "\n"
                + "Axis 1: " + this.getRawAxis(1) + "\n"
                + "Axis 2: " + this.getRawAxis(2) + "\n"
                + "Axis 3: " + this.getRawAxis(3) + "\n"
                + "Axis 4: " + this.getRawAxis(4) + "\n"
                + "Axis 5: " + this.getRawAxis(5) + "\n"
                + "Axis 6: " + this.getRawAxis(6);
        
        return out;
    }
}
