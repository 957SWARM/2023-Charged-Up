package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBoardLeft extends Joystick{

    public ButtonBoardLeft(int port) {
        super(port);
    }

    final int blingCube = 4;
    final int blingCone = 3;
    final int extra = 2; 
    final int pickupCone = 1;

    final int armRectract = 12;
    final int armMid = 11;
    final int armMax = 10;
    final int armSub = 9;

    public boolean blingCube(){
        return super.getRawButton(blingCube);
    }

    public boolean blingCone(){
        return super.getRawButton(blingCone);
    }

    public boolean extra(){
        return super.getRawButton(extra);
    }

    public boolean pickupCone(){
        return super.getRawButton(pickupCone);
    }

    public boolean armRectract(){
        return super.getRawButton(armRectract);
    }

    public boolean armMid(){
        return super.getRawButton(armMid);
    }

    public boolean armMax(){
        return super.getRawButton(armMax);
    }

    public boolean armSub(){
        return super.getRawButton(armSub);
    }


}