package frc.robot;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.ShooterSpeed;
import frc.robot.Constants.WristPositions;

public class VisionSubsystem {

    // Global variables
    private final SlewRateLimiter m_slewTX = new SlewRateLimiter(3);
	private final SlewRateLimiter m_slewTY = new SlewRateLimiter(3);
    int aprilTagCase = 0;
    int retroTapeCase = 0;
	int positionCase = 0;
	int coneCase = 0;
    double aprilTagTX = 0;
	double aprilTagTY = 0;
    double retroTapeTX = 0;

	double ATsum = 0;
	double ATruns = 0;
	double ATavgTX = 0;
	double RTsum = 0;
	double RTruns = 0;
	double RTavgTX = 0;

	double ATXsum = 0;
	double ATYsum = 0;

	public double MAX_TIME = 0.3;
	public double lockOnTimer = 0;
	public double[] cubePosition = {0, 0};
	double manipulateCubeTimer = 0;

    // Constants
    // limelight .178 meters to the left of robot's center
    double offset = .178;
    

    public VisionSubsystem(){

    }

    // April Tags
    public void maintainTX(Limelight limelight){
        aprilTagTX = m_slewTX.calculate(limelight.getAlignmentOffset()/23)*23;
    }

	public void maintainTY(Limelight limelight){
        aprilTagTY = m_slewTY.calculate(limelight.getDistanceOffset()/23)*23;
    }

    public void resetCases(){
        aprilTagCase = 0;
        retroTapeCase = 0;
		positionCase = 0;
    }
	
    public boolean AprilTagTracking(Drivetrain swerve, Limelight limelight){

        switch(aprilTagCase){
			//Case 0 - Get TX
			case 0:
				swerve.drive(0, 0, 0, false);
				if (limelight.getTv() == 1){
					ATsum += aprilTagTX;
					ATruns += 1;
					if(ATruns == 5){
						ATavgTX = ATsum / 5;
						aprilTagCase = 1;
						swerve.resetOdometry();
					}
				}
			break;
			
			//Case 1 - Drive
			case 1:

			break;
			
			//Case 2 - Hit wall
			case 2:
				if( swerve.hitwall()){
					aprilTagCase = 0;
                    return true;
                }
            break;
		}

        return false;
    }

    public boolean TapeTracking(WristPositions wristPosition, Drivetrain swerve, Limelight limelight, Wrist wrist){
        limelight.setPipe(2);
        switch(retroTapeCase){
			//Case 0 - PID Track
			case 0:
				if(swerve.trackTapePID(limelight.getTx(), 1.5, .1)){
					// wrist.set(wristPosition);
					//retroTapeCase = 1;
					// swerve.resetOdometry();
				}
			break;
			
			//Case 1 - Drive to wall
			case 1:
				if ( swerve.hitwall()){
					retroTapeCase = 0;
					limelight.setPipe(0);
                    return true;
				}
			break;
		}
        return false;
    }

	public boolean lockOnApriltag(Drivetrain swerve, Limelight limelight, double a){

		swerve.drive(0, 0, 0, false);

		if(lockOnTimer > MAX_TIME) {
			if (limelight.getTv() == 1){
				ATruns  += 1;
				ATXsum += limelight.campose[0];
				ATYsum += limelight.campose[2];
				if(ATruns == 5){
					swerve.resetOdometry();
					cubePosition[0] = ATXsum / ATruns;
					cubePosition[1] = ATYsum / ATruns + a;
					return true;
				}
			}
			
		} else {

			ATXsum = 0;
			ATYsum = 0;
			ATruns = 0;
		}

		if(true) {
			lockOnTimer += 0.02;
		}

		return false;
	}

	// Drives to be in line with April Tag plus offset in meters
	public boolean positionFromAT(Drivetrain swerve, Limelight limelight, double offset){
		limelight.setPipe(0);
		switch(positionCase){

			// Get a set position for the robot
			case 0:
				if(lockOnApriltag(swerve, limelight, .75)){	
					positionCase = 1;
				}

			break;
			
			// Align with april tag plus offset
			case 1:
				lockOnTimer = 0;
				if(swerve.driveToPosition(0, cubePosition[0] + offset, 1, 0.05)){
					positionCase = 0;
					return true;
				}

			break;


		}
		return false;
	}

}
