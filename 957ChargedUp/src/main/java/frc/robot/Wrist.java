package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.WristPositions;

public class Wrist{

    CANSparkMax m_wristMotor = new CANSparkMax(10, MotorType.kBrushless);
    
    SparkMaxPIDController m_wristPIDController = m_wristMotor.getPIDController();
    RelativeEncoder wristEncoder = m_wristMotor.getEncoder();

    PIDController m_wristNewPID = new PIDController(1, 0, 0);
    
    double targetPosition = 0;
    WristPositions wristPos = WristPositions.retract;
    IdleMode currentIdleMode = IdleMode.kCoast;

    public Wrist(){
        m_wristMotor.restoreFactoryDefaults();
        wristEncoder = m_wristMotor.getEncoder();

        m_wristMotor.setSmartCurrentLimit(30);

        m_wristPIDController.setP( 5e-5);
        m_wristPIDController.setI(1e-6);
        m_wristPIDController.setD(0);
        m_wristPIDController.setOutputRange(-1, 1);
        m_wristPIDController.setSmartMotionMaxVelocity(900, 0);
        m_wristPIDController.setSmartMotionMaxAccel(900, 0);


    }

    public void set(WristPositions position){
       targetPosition = position.wristPosition();
       wristPos = position;
    }

    public String getLabel(){
        return wristPos.text();
    }
    public void run(double clawPosition){
        m_wristNewPID.calculate(targetPosition, targetPosition);
     // m_wristPIDController.setReference(targetPosition, ControlType.kSmartMotion);
        // m_wristPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, 0, calculateFF(wristEncoder.getPosition()), SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }
}