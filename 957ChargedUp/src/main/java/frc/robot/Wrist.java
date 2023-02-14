package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class Wrist{

    CANSparkMax m_wristMotor = new CANSparkMax(0, MotorType.kBrushless);
    SparkMaxPIDController m_wristPIDController = m_wristMotor.getPIDController();
    AbsoluteEncoder m_wristEncoder;
    double targetPosition = 0;
    double defaultPosition = 0;

    public Wrist(){
        m_wristMotor.restoreFactoryDefaults();
        m_wristPIDController.setFeedbackDevice(m_wristEncoder);
        m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        m_wristPIDController.setP(0);
        m_wristPIDController.setI(0);
        m_wristPIDController.setD(0);

        m_wristPIDController.setPositionPIDWrappingEnabled(true);
    }

    public void set(WristPositions position){
       targetPosition = position.wristPosition();
    }
    public void run(){
        m_wristPIDController.setReference(targetPosition + defaultPosition, CANSparkMax.ControlType.kPosition);
    }
}