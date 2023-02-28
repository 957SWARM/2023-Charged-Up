package frc.robot.Constants;
public enum ShooterSpeed {

    placeCube(0.2  , "place cube"),
    placeCone(0.6, "place cone"),
    teleopEject(0.7 , "Teleop Eject"),
    midCube(.5, "Mid Cube"),
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