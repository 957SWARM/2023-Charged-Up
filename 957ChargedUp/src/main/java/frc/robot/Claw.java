package frc.robot;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw {

    DoubleSolenoid clawDoubleSolenoid = new DoubleSolenoid(5555, PneumaticsModuleType.CTREPCM, 0, 1);
    TalonSRX clawMain = new TalonSRX(111);
    TalonSRX clawFollower = new TalonSRX(2222);
    DigitalInput m_limitSwitch = new DigitalInput(0);

    public Claw(){
        clawFollower.follow(clawMain);
        clawFollower.getInverted();
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
        if(!m_limitSwitch.get()){
            clawMain.set(TalonSRXControlMode.PercentOutput, -speed);
        }else{
            clawStop();
        }
    } 

    public void clawOutake(double speed){
        clawMain.set(TalonSRXControlMode.PercentOutput, speed);
    }
    
    public void clawStop(){
        clawMain.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
}
