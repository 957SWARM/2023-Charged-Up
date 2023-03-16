package frc.robot;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ShooterSpeed;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw {

    DoubleSolenoid clawDoubleSolenoid = new DoubleSolenoid(14, PneumaticsModuleType.REVPH, 0, 1);
    TalonSRX clawMain = new TalonSRX(11);
    TalonSRX clawFollower = new TalonSRX(12);
    DigitalInput m_limitSwitch = new DigitalInput(0);

    public Claw(){
        clawFollower.follow(clawMain);
        clawFollower.setInverted(true);
        clawMain.setNeutralMode(NeutralMode.Brake);
        clawFollower.setNeutralMode(NeutralMode.Brake);
        clawFollower.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		// clawMain.setSensorPhase(true);

    }
    
    public void coneMode(){
        if(clawDoubleSolenoid.get() != Value.kForward)
        clawDoubleSolenoid.set(Value.kForward);
    }
    public void cubeMode(){
        if(clawDoubleSolenoid.get() != Value.kReverse)
        clawDoubleSolenoid.set(Value.kReverse);
    }

    public void clawIntake(double speed){
        if(m_limitSwitch.get()){
            clawMain.set(TalonSRXControlMode.PercentOutput, -speed);
        }else{
            clawMain.set(TalonSRXControlMode.PercentOutput, -.2);
        }
    } 

    public void clawOuttake(ShooterSpeed speed){
        clawMain.set(TalonSRXControlMode.PercentOutput, speed.speed());
    }
    
    public void clawStop(){
        clawMain.set(TalonSRXControlMode.PercentOutput, -.05);
    }
    
    public double clawPosition(){
        return clawFollower.getSelectedSensorPosition();
    }
}
