package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Wing extends DoubleSolenoid{

    public Wing(int port1, int port2){
        super(14, PneumaticsModuleType.REVPH, port1, port2);
    }

    public void extendWing(){
        // System.out.println("Extend");
        if(super.get() != Value.kForward)
            super.set(Value.kForward);
    }

    public void retractWing(){
        // System.out.println("Retract");
        if(super.get() != Value.kReverse)
            super.set(Value.kReverse);
    }

    public void toggleWing(){
        if(super.get() == Value.kReverse){
            super.set(Value.kForward);
        }
        else if(super.get() == Value.kForward){
            super.set(Value.kReverse);
        }
    }
}
