package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MoveFourBars;
import frc.robot.Constants.WristPositions;

public class Shuffleboard {
  boolean autoModeSet = false;
  SendableChooser<String> m_autoChooser = new SendableChooser<>();
  static Boolean cubeMode = false;
  static Boolean turnMode = false;
  double takenIntakeFudge = 0;
  

  // Run when shuffleboard is first initialized
  public Shuffleboard(){


    // Add options
    m_autoChooser.setDefaultOption("mid", "mid");
    m_autoChooser.addOption("Cone Left", "coneLeft");
    m_autoChooser.addOption("Cone Right", "coneRight");
    m_autoChooser.addOption("Do Nothing", "doNothing");
    m_autoChooser.addOption("Only Place Cube", "placeDoNothingCube");
    m_autoChooser.addOption("Only Place Cone", "placeDoNothingCone");
    m_autoChooser.addOption("crossTheLinePlease", "crossTheLine");

    SmartDashboard.putData(m_autoChooser);

    // Put chooser on dashboard

    SmartDashboard.putNumber("Intake Fudge", 0);


  }


  public void updateShuffleboard(Drivetrain d, Claw c, String wp, String mfb, VisionSubsystem v, int speedVar) 
    {  

        //BLING BOARD (placeholder value)
        SmartDashboard.putString("Bling Board", "bling");

        //VISION TRAKCING ?!?!?!
        // SmartDashboard.putBoolean("Vision Tracking?", .visionControlled);

        //ARM AND WRIST POSITIONS
        SmartDashboard.putString("Wrist Position", wp);
        SmartDashboard.putString("Arm Position", mfb);

        //CLAW OR CONE?
        if(c.clawDoubleSolenoid.get() == Value.kForward)
            cubeMode = false;
        if(c.clawDoubleSolenoid.get() != Value.kForward)
            cubeMode = true;
		SmartDashboard.putBoolean("Claw Mode", cubeMode);


      //ARE WE IN TURN MODE?
        if(speedVar == 0 || speedVar == 1){
        	turnMode = false;
        }else if(speedVar == 2 || speedVar == 3)
			turnMode = true;
	    SmartDashboard.putBoolean("Turn Mode", turnMode);

        
        //BOT ANGLE AND WHEEL ANGLES
        SmartDashboard.putNumber("Bot Angle", d.m_navx.getAngle());
        SmartDashboard.putNumber("FL", d.getFrontLeftSwerveModuleAngle());
        SmartDashboard.putNumber("FR", d.getFrontRightSwerveModuleAngle());
        SmartDashboard.putNumber("BL", d.getBackLeftSwerveModuleAngle());
        SmartDashboard.putNumber("BR", d.getBackRightSwerveModuleAngle());

        // Limit Switches
        SmartDashboard.putBoolean("Wrist Limit Switch", c.m_limitSwitch.get());
        
        // VISION
        SmartDashboard.putNumber("X position", d.xPosition);
        SmartDashboard.putNumber("Target", v.cubePosition[0]);

        SmartDashboard.putNumber("Gyro Roll", d.m_navx.getRoll());

        takenIntakeFudge = SmartDashboard.getNumber("Intake Fudge", 0);
    }

  public String updateAuto(){
    return m_autoChooser.getSelected();
  }
}
