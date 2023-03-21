package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;


public class ButtonBoardRightNew extends Joystick{

    public ButtonBoardRightNew(int port) {
        super(port);
    }
    
    final int highCube = 10;
    final int midCube = 9;

    final int fourBarMid = 11;
    
    final int doubleSubstation = 12;

    final int vision1 = 2;
    final int vision2 = 1;
    final int vision3 = 3;

    final int cubePickup = 4;

    public boolean highCube(){
        return super.getRawButton(highCube);
    }

     public boolean midCube(){
        return super.getRawButton(midCube);
    }

     public boolean fourBarMid(){
        return super.getRawButton(fourBarMid);
    }

    public boolean doubleSubstation(){
        return super.getRawButton(doubleSubstation);
    }

    // Vision Left
    public boolean vision1(){
        return super.getRawButton(vision1);
    }

    // Vision Cube
    public boolean vision2(){
        return super.getRawButton(vision2);
    }

    // Vision Right
    public boolean vision3(){
        return super.getRawButton(vision3);
    }

    public boolean cubePickup(){
        return super.getRawButton(cubePickup);
    }
    }