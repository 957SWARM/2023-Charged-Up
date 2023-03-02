package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MoveFourBars;
import frc.robot.Constants.WristPositions;

public class Shuffleboard {
  boolean autoModeSet = false;
  SendableChooser<String> m_chooser = new SendableChooser<>();
  Boolean cubeModeQuestionMark = false;
  Boolean turnModeQuestionMark = false;
  

  // Run when shuffleboard is first initialized
  public Shuffleboard(){


    // Add options

    // Put chooser on dashboard
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putBoolean("Claw Mode", cubeModeQuestionMark);

    SmartDashboard.putString("Ally 1", "0");
    SmartDashboard.putString("Ally 2", "0");

    SmartDashboard.putBoolean("Turn Mode", turnModeQuestionMark);
  }


  public void updateShuffleboard(int cargo, Drivetrain d, Claw c, WristPositions wp, MoveFourBars mfb, int speedVar) 
    {
        //ARM AND WRIST POSITIONS
        SmartDashboard.putString("Wrist Position", wp.text());
        SmartDashboard.putString("Arm Position", mfb.text());

        //CLAW OR CONE?
        if(c.clawDoubleSolenoid.get() == Value.kForward)
            cubeModeQuestionMark = false;
        if(c.clawDoubleSolenoid.get() != Value.kForward)
            cubeModeQuestionMark = true;

		//ARE WE IN TURN MODE?
        if(speedVar == 0 || speedVar == 1){
          turnModeQuestionMark = false;
        }else if(speedVar == 2 || speedVar == 3)
			turnModeQuestionMark = true;
        
        //BOT ANGLE AND WHEEL ANGLES
        SmartDashboard.putNumber("Bot Angle", d.m_navx.getAngle());
        SmartDashboard.putNumber("FL", d.getFrontLeftSwerveModuleAngle());
        SmartDashboard.putNumber("FR", d.getFrontRightSwerveModuleAngle());
        SmartDashboard.putNumber("BL", d.getBackLeftSwerveModuleAngle());
        SmartDashboard.putNumber("BR", d.getBackRightSwerveModuleAngle());
        

        
    }

  public String updateAuto(){
    System.out.println(m_chooser.getSelected());
    return m_chooser.getSelected();
  }
}
