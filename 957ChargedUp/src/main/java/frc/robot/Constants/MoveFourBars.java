package frc.robot.Constants;

public enum MoveFourBars{
    //bar height levels
    substation(15, "Substation Position"), 
    ground( 0.1, "Ground Position"), //IF YOU CHANGE THIS, ALSO CHANGE LIMIT SWITCH VALUE PLEASE
    mid( 31, "Mid Position"), 
    high(46, "High Position");
    
    private final double barTargetSetpoint;
    private final String m_identifier;

    private MoveFourBars(double rotateBars, String identifer){
        barTargetSetpoint = rotateBars;
        m_identifier = identifer;
    }

    public double barPosition(){
        return barTargetSetpoint;
    }

    public String text(){
        return m_identifier;
    }


}
