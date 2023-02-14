package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonPanel {
    private final Joystick controller1 = new Joystick(1);
    private final Joystick controller2 = new Joystick(2);
    
    //buttons
    final int wristRetract = 0;
    final int wristScoreUp = 0;
    final int wristScoreOut = 0;
    final int wristScoreGround = 0;

    final int armHigh = 0;
    final int armMid = 0;
    final int armGround = 0;
    final int armSubStation = 0;

    final int blingCone = 0;
    final int blingCube = 0;
    

    public boolean wristRetractPressed(){
        return controller1.getRawButton(wristRetract);
    }

    public boolean wristScoreUpPressed(){
        return controller1.getRawButton(wristScoreUp);
    }

    public boolean wristScoreOutPressed(){
        return controller1.getRawButton(wristScoreOut);
    }

    public boolean wristGroundPressed(){
        return controller1.getRawButton(wristScoreGround);
    }

    public boolean armHighPressed(){
        return controller1.getRawButton(armHigh);
    }

    public boolean armMidPressed(){
        return controller1.getRawButton(armMid);
    }

    public boolean armGroundPressed(){
        return controller1.getRawButton(armGround);
    }

    public boolean armSubPressed(){
        return controller1.getRawButton(armSubStation);
    }

    public boolean blingConePressed(){
        return controller1.getRawButton(blingCone);
    }  

    public boolean blingCubePressed(){
        return controller1.getRawButton(blingCube);
    } 

}
