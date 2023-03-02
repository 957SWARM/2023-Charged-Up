package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.WristPositions;

public class Wrist{

    CANSparkMax m_wristMotor = new CANSparkMax(10, MotorType.kBrushless);
    
    SparkMaxPIDController m_wristPIDController = m_wristMotor.getPIDController();
    RelativeEncoder wristEncoder = m_wristMotor.getEncoder();

    
    double targetPosition = 0;
    IdleMode currentIdleMode = IdleMode.kCoast;

    public Wrist(){
        m_wristMotor.restoreFactoryDefaults();
        wristEncoder = m_wristMotor.getEncoder();

        m_wristMotor.setSmartCurrentLimit(30);

        m_wristPIDController.setP( 5e-5);
        m_wristPIDController.setI(1e-6);
        m_wristPIDController.setD(0);
        m_wristPIDController.setFF(0.000156);
        m_wristPIDController.setOutputRange(-1, 1);
        m_wristPIDController.setSmartMotionMaxVelocity(900, 0);
        m_wristPIDController.setSmartMotionMaxAccel(900, 0);


    }

    public double calculateFF(double encoderPosition){
        double angle = ((encoderPosition /40) * 360) + 35;
        double combinedArm = 100;

        double motorOhms = 36/1000;
        double motorTorque = 23;
        double motorStallCurrent = 105;

        double gearbox = 20;
        double Kt = (motorTorque/motorStallCurrent);

        double kF = -( (combinedArm * motorOhms) / (Kt * gearbox) * Math.cos(Math.toRadians(angle)));
        kF = 0;
        return kF;
    }

    public void set(WristPositions position){
       targetPosition = position.wristPosition();
    }
    public void run(){
       m_wristPIDController.setReference(targetPosition, ControlType.kSmartMotion);
        // m_wristPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, 0, calculateFF(wristEncoder.getPosition()), SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }
}