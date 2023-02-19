package frc.robot;
public enum WristPositions {

    retract(0.1, "retract"),
    scoreUp(2.5, "scoreUp"),
    scoreOut(7, "scoreOut"),
    ground(9,"Ground");

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
