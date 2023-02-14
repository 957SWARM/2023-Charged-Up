package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ArmPidConstants;
import frc.robot.Constants.FourBarPidConstants;

public class FourBar {
    CANSparkMax fourBarMotor = new CANSparkMax(9999999, MotorType.kBrushless);
    
    SparkMaxPIDController fourBarPid = fourBarMotor.getPIDController();
    RelativeEncoder fourBarEncoder = fourBarMotor.getEncoder();

    //CANSparkMax armMotor = new CANSparkMax(88888888, MotorType.kBrushless);

    //SparkMaxPIDController armPid = armMotor.getPIDController();
    //RelativeEncoder armEncoder = armMotor.getEncoder();

    //double targetArmPosition = 0;
    double targetBarPosition = 0;


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


        //armMotor.restoreFactoryDefaults();
        //armMotor.setIdleMode(IdleMode.kBrake);

        /*
        armPid.setP(ArmPidConstants.kArmP);
        armPid.setI(ArmPidConstants.kArmI);
        armPid.setD(ArmPidConstants.kArmD);
        armPid.setFF(ArmPidConstants.kArmFF);
        armPid.setOutputRange(ArmPidConstants.kArmMinOutput, ArmPidConstants.kArmMaxOutput);
        armPid.setSmartMotionMaxVelocity(ArmPidConstants.armMaxVel, 0);
        armPid.setSmartMotionMaxVelocity(ArmPidConstants.armMaxAcc, 0);
        */

    }
    
    public void setLevel(MoveFourBars level) {
	    //targetArmPosition = level.armPosition();
        targetBarPosition = level.barPosition();
	}

    public void run(){
        /*
        if(targetArmPosition < 0.1){
            targetArmPosition = 0.1;
        }
        if(targetArmPosition > 100){
            targetArmPosition = 100;
        }
        */
        if(targetBarPosition < 0.1){
            targetBarPosition = 0.1;
        }
        if(targetBarPosition > 100){
            targetBarPosition = 100;
        }
    
       // armPid.setReference(targetArmPosition, ControlType.kSmartMotion);
        fourBarPid.setReference(targetBarPosition, ControlType.kSmartMotion);
    }
}
       