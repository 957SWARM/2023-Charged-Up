package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw {

    DoubleSolenoid clawDoubleSolenoid = new DoubleSolenoid(5555, PneumaticsModuleType.CTREPCM, 0, 1);
    AnalogInput clawSensor = new AnalogInput(44444);

    public void openClaw(){
        if(clawDoubleSolenoid.get() != Value.kForward)
        clawDoubleSolenoid.set(Value.kForward);
    }
    public void closeClaw(){
        if(clawDoubleSolenoid.get() != Value.kReverse)
        clawDoubleSolenoid.set(Value.kReverse);
    }

}
