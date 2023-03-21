package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Constants.FourBarPidConstants;
import frc.robot.Constants.MoveFourBars;

public class FourBar {
    CANSparkMax fourBarMotor = new CANSparkMax(9, MotorType.kBrushless);
    
    SparkMaxPIDController fourBarPid = fourBarMotor.getPIDController();
    RelativeEncoder fourBarEncoder = fourBarMotor.getEncoder();
    DigitalInput m_limitSwitch = new DigitalInput(1);
    boolean oldLimitSwitchValue = true;
    //CANSparkMax armMotor = new CANSparkMax(88888888, MotorType.kBrushless);

    //SparkMaxPIDController armPid = armMotor.getPIDController();
    //RelativeEncoder armEncoder = armMotor.getEncoder();

    //double targetArmPosition = 0;
    double targetBarPosition = 0;
    IdleMode currentIdleMode = IdleMode.kCoast;
    MoveFourBars barPosition = MoveFourBars.ground;

    public FourBar(){        
        fourBarMotor.restoreFactoryDefaults();
        fourBarMotor.setIdleMode(IdleMode.kBrake);

        fourBarPid.setP(FourBarPidConstants.kFourBarP);
        fourBarPid.setI(FourBarPidConstants.kFourBarI);
        fourBarPid.setD(FourBarPidConstants.kFourBarD);
        fourBarPid.setFF(FourBarPidConstants.kFourBarFF);
        fourBarPid.setOutputRange(FourBarPidConstants.kFourBarMinOutput, FourBarPidConstants.kFourBarMaxOutput);
        fourBarPid.setSmartMotionMaxVelocity(FourBarPidConstants.barMaxVel, 0);
        fourBarPid.setSmartMotionMaxAccel(FourBarPidConstants.barMaxAcc, 0);

        fourBarMotor.setSmartCurrentLimit(30);

       
    }
    
    public void setLevel(MoveFourBars level) {
        targetBarPosition = level.barPosition();
        barPosition = level;
	}

    public void run(){
      
        if(m_limitSwitch.get() && m_limitSwitch.get() != oldLimitSwitchValue){
            //fourBarEncoder.setPosition(0.1);
        }
        oldLimitSwitchValue = m_limitSwitch.get();

        if(targetBarPosition < 0){
            targetBarPosition = 0;
        }
        if(targetBarPosition > 100){
            targetBarPosition = 100;
        }
        if(targetBarPosition == 0 && fourBarEncoder.getPosition() < 1){
            // fourBarMotor.set(0);
            return;
        }
        if(fourBarMotor.getIdleMode() != IdleMode.kCoast){
            fourBarMotor.setIdleMode(IdleMode.kCoast);
        }
        fourBarPid.setReference(targetBarPosition, ControlType.kSmartMotion);
    }

    public String getLabel(){
        return barPosition.text();
    }
    
}
       