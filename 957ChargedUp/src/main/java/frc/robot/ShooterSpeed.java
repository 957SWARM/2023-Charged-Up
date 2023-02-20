package frc.robot;
public enum ShooterSpeed {

    place(0.1  , "place"),
    teleopEject(0.7 , "Teleop Eject"),
    midCube(0, "Mid Cube"),
    highCube(0, "High Cube");

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