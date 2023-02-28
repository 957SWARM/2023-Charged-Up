package frc.robot;
public enum WristPositions {

    retract(0.1  , "retract"),
    scoreUp(9, "scoreUp"),//9
    scoreOut(14, "scoreOut"),
    highCube(7, "highCube"),
    midCube(12, "midCube"),
    substation(12.5, "substation"),
    ground(18,"Ground");

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
