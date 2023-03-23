package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;

public class NavController extends Joystick {

    public NavController(int port) {
        super(port);
    }

    final int a = 1;
    final int b = 2;
    final int c = 3;
    final int d = 4;

    public boolean cubeShootA(){
        return super.getRawButton(a);
    }
    public boolean cubeShootB(){
        return super.getRawButton(b);
    }
    public boolean cubeShootC(){
        return super.getRawButton(c);
    }
    public boolean cubeShootD(){
        return super.getRawButton(d);
    }
    
}
