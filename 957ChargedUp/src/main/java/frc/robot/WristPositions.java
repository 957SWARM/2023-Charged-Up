package frc.robot;
public enum WristPositions {

    retract(0, "retract"),
    scoreUp(0, "scoreUp"),
    scoreOut(0, "scoreOut"),
    ground(0,"Ground");

    private final double wristTargetSetpoint;
    private final String m_identifer;

    private WristPositions(double position, String identifer){
        wristTargetSetpoint = position;
        m_identifer = identifer;
    }

    public double wristPosition(){
        return wristTargetSetpoint;
    }

    public String text(){
        return m_identifer;
    }

}
