package frc.robot.Constants;
public enum WristPositions {

    retract(10 , "retract"), //IF YOU CHANGE THIS, ALSO CHANGE LIMIT SWITCH VALUE PLEASE
    scoreUp(1060, "scoreUp"),   //9
    scoreSub(1150, "scoreSub"),
    scoreOut(1520, "scoreOut"),
    highCube(9, "highCube"),
    midCube(14, "midCube"),
    substation(14.5, "substation"),
    coneGround(2000, "cone ground"),
    backShootCube(300, "back shoot"),
    cubeGround(1950,"cube ground"); // 1850 for auto????


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
