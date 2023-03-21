package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.WristPositions;

public class Wrist{

    CANSparkMax m_wristMotor = new CANSparkMax(10, MotorType.kBrushless);
    
    SparkMaxPIDController m_wristPIDController = m_wristMotor.getPIDController();
    RelativeEncoder wristEncoder = m_wristMotor.getEncoder();
    DigitalInput m_limitSwitch = new DigitalInput(2);

    // d: 0.000375
    PIDController m_wristNewPID = new PIDController(0.00165, 0.00015, 0.0000);
    
    double targetPosition = 0;
    WristPositions wristPos = WristPositions.retract;
    IdleMode currentIdleMode = IdleMode.kCoast;

    boolean oldLimitSwitchValue = true;

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

    public void run(double clawPosition, double maxOut){

        if(m_limitSwitch.get() && m_limitSwitch.get() != oldLimitSwitchValue){
            oldLimitSwitchValue = m_limitSwitch.get();
            wristEncoder.setPosition(10);
        }

        double output = m_wristNewPID.calculate(clawPosition, targetPosition);
        if (output > maxOut){
            output = maxOut;
        }
        else if (output < -maxOut){
            output = -maxOut;
        }

        if(wristEncoder.getPosition() >= 800){
            m_wristMotor.set(output - .15);
        }else{
            m_wristMotor.set(output);
        }

        // m_wristPIDController.setReference(targetPosition, ControlType.kSmartMotion);
        // m_wristPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, 0, calculateFF(wristEncoder.getPosition()), SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }
}