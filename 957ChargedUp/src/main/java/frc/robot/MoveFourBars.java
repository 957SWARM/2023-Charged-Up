package frc.robot;

public enum MoveFourBars{
    //bar height levels
    pickup(00, 00, "Pickup Position"), 
    low(0.1, 0.0, "Low Position"), 
    medium(11, 111, "Medium Position"), 
    high(22, 222, "High Position");
    
    private final double armTargetSetpoint;
    private final double barTargetSetpoint;
    private final String m_identifier;

    private MoveFourBars(double moveArm, double rotateBars, String identifer){
        armTargetSetpoint = moveArm;
        barTargetSetpoint = rotateBars;
        m_identifier = identifer;
    }

    public double armPosition(){
        return armTargetSetpoint;
    }

    public double barPosition(){
        return barTargetSetpoint;
    }

    public String text(){
        return m_identifier;
    }


}
