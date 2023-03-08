package frc.robot.Constants;
public enum WristPositions {

    retract(.1  , "retract"),
    scoreUp(11, "scoreUp"),//9
    scoreOut(16, "scoreOut"),
    highCube(9, "highCube"),
    midCube(14, "midCube"),
    substation(14.5, "substation"),
    coneGround(18, "cone groudn"),
    backShootCube(3, "back shoot"),
    cubeGround(20,"cube ground");


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
