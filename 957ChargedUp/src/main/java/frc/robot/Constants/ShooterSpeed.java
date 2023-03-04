package frc.robot.Constants;
public enum ShooterSpeed {

    placeCube(0.2  , "place cube"),
    teleopEject(.5 , "Teleop Eject"), //.7
    midCube(.5, "Mid Cube"),
    midShootAuto(.5, "midShootAuto"),
    highCube(.7, "High Cube");

    private final double speed;
    private final String m_identifer;

    private ShooterSpeed(double s, String identifer){
        speed = s;
        m_identifer = identifer;
    }

    public double speed(){
        return speed;
    }

    public String text(){
        return m_identifer;
    }

}