package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonPanel {
    private final Joystick controller1 = new Joystick(1);
    private final Joystick controller2 = new Joystick(2);
    
    //buttons
    final int wristRetract = 10;
    final int wristScoreUp = 7;
    final int wristScoreOut = 8;
    final int wristScoreGround = 9;

    final int armHigh = 4;
    final int armMid = 5;
    final int armGround = 6;
    final int armSubStation = 3;

    final int blingCone = 2;
    final int blingCube = 1;
    

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
