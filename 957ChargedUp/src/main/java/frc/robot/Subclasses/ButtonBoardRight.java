package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBoardRight extends Joystick {

    public ButtonBoardRight(int port) {
        super(port);
    }

    final int wristRetract = 9;
    final int wrist45 = 10;
    final int wristOut = 11;
    final int wristSub = 12;

    final int vision1 = 2;
    final int vision2 = 1;
    final int vision3 = 3;
    
    final int pickupCube = 4;

    public boolean wristRetract(){
        return super.getRawButton(wristRetract);
    }

    public boolean wrist45(){
        return super.getRawButton(wrist45);
    }

    public boolean wristOut(){
        return super.getRawButton(wristOut);
    }

    public boolean wristSub(){
        return super.getRawButton(wristSub);
    }
    
    public boolean vision1(){
        return super.getRawButton(vision1);
    }
    
    public boolean vision2(){
        return super.getRawButton(vision2);
    }

    public boolean vision3(){
        return super.getRawButton(vision3);
    }

    public boolean pickupCube(){
        return super.getRawButton(pickupCube);
    }
}
