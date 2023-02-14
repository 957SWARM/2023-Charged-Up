package frc.robot;

public enum MoveFourBars{
    //bar height levels
    substation(00, 00, "Substation Position"), 
    ground(0.1, 0.0, "Ground Position"), 
    mid(11, 111, "Mid Position"), 
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
