package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;

public class DriveController extends Joystick {

    public DriveController(int port) {
        super(port);
        //TODO Auto-generated constructor stub
    }
    
    static int xAxisDrive = 0; // 0 axis
	static int yAxisDrive = 1; // 1 axis
	static int gTurnAxis = 4; // 4 axis 
    static int rightStickY = 5; // 5 axis
	static int clawToggle = 3; // button x
	static int shifter = 1; // button a
	static int driveStyle = 8; // right face button 
	static int clawIntake = 2; // trigger left axis 2
	static int midCube = 5; // left face
	static int place = 4; // y button
    static int highCube = 6; //right bumper 
	static int clawIntakeStop = 3; // left trigger axis 3

    public boolean clawToggle(){
        return super.getRawButton(clawToggle);
    }

    public boolean shifter(){
        return super.getRawButton(shifter);
    }

    public boolean driveStyle(){
        return super.getRawButton(driveStyle);
    }

    public boolean midCube(){
        return super.getRawButton(midCube);
    }

    public boolean highCube(){
        return super.getRawButton(highCube);
    }

    public boolean place(){
        return super.getRawButton(place);
    }

    public double getRightStickX(){
        return super.getRawAxis(gTurnAxis);
    }

    public double getRightStickY(){
        return super.getRawAxis(rightStickY);
    }

    public double getLeftStickX(){
        return super.getRawAxis(xAxisDrive);
    }

    public double getLeftStickY(){
        return super.getRawAxis(yAxisDrive);
    }

    public double clawIntake(){
        return super.getRawAxis(clawIntake);
    }

    public double clawIntakeStop(){
        return super.getRawAxis(clawIntakeStop);
    }
    
}
