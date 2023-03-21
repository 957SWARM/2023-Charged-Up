package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;


public class ButtonBoardLeftNew extends Joystick{

    public ButtonBoardLeftNew(int port) {
        super(port);
    }

    // Placeholder button port values
    final int blingCone = 3;
    final int blingCube = 4;

    final int retractArm = 2;
    final int retractWrist = 10;

    final int wristOut = 11;
    final int wrist45 = 12;

    final int conePickup = 1;
    final int singleSub = 9;


    // Bling functions
    public boolean blingCone(){
        return super.getRawButton(blingCone);
    }
    public boolean blingCube(){
        return super.getRawButton(blingCube);
    }

    // Retract functions
    public boolean retractArm(){
        return super.getRawButton(retractArm);
    }
    public boolean retractWrist(){
        return super.getRawButton(retractWrist);
    }

    // Wrist funtions
    public boolean wrist45(){
        return super.getRawButton(wrist45);
    }
    public boolean wristOut(){
        return super.getRawButton(wristOut);
    }

    // Pickup functions
    public boolean conePickup(){
        return super.getRawButton(conePickup);
    }
    public boolean singleSub(){
        return super.getRawButton(singleSub);
    }

}
