package frc.robot.Constants;
public enum WristPositions {

    retract(11 , "retract"),
    scoreUp(1060, "scoreUp"),//9
    scoreOut(1520, "scoreOut"),
    highCube(9, "highCube"),
    midCube(14, "midCube"),
    substation(14.5, "substation"),
    coneGround(2050, "cone groudn"),
    backShootCube(3, "back shoot"),
    cubeGround(1800,"cube ground");


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
