package frc.robot;
import frc.robot.Drivetrain;
import frc.robot.Limelight;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class VisionSubsystem {

    // Global variables
    private final SlewRateLimiter m_slewTX = new SlewRateLimiter(3);
	private final SlewRateLimiter m_slewTY = new SlewRateLimiter(3);
    int aprilTagCase = 0;
    int retroTapeCase = 0;
	int cubeCase = 0;
    double aprilTagTX = 0;
	double aprilTagTY = 0;
    double retroTapeTX = 0;

	double ATsum = 0;
	double ATruns = 0;
	double ATavgTX = 0;
	double RTsum = 0;
	double RTruns = 0;
	double RTavgTX = 0;

	double ATXruns = 0;
	double ATYruns = 0;
	double ATXsum = 0;
	double ATYsum = 0;

	public double MAX_TIME = 0.3;
	public double lockOnTimer = 0;
	public double[] cubePosition;

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
				if( swerve.realestDriveMeters(ATavgTX + offset, .7, .05)){
					aprilTagCase = 2;
                }
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

    public boolean TapeTracking(Drivetrain swerve, Limelight limelight){
        
        switch(retroTapeCase){
			//Case 0 - PID Track
			case 0:
				if(swerve.trackTapePID(limelight.getTx(), 1.5, .1)){
					retroTapeCase = 1;
					swerve.resetOdometry();
				}
			break;
			
			//Case 1 - Drive x meters (adjusting for limelight position on robot)
			case 1:
				if (swerve.loserDriveMeters(offset, .5, .05)){
					retroTapeCase = 2;
				}
			break;
			
			//Case 2 - Drive to wall
			case 2:
				if ( swerve.hitwall()){
					retroTapeCase = 0;
                    return true;
				}
			break;
		}
        return false;
    }

	public double[] lockOnApriltag(Drivetrain swerve, Limelight limelight, double a){

		swerve.drive(0, 0, 0, false);

		if(lockOnTimer > MAX_TIME) {

			if (limelight.getTv() == 1){
				ATXsum  += 1;
				ATYsum  += 1;
				ATXruns += aprilTagTX;
				ATYruns += aprilTagTY;
				if(ATXruns == 5){
					swerve.resetOdometry();
					return new double[] {-ATXsum / ATYruns, -ATYsum / ATXruns + a};
				}
			}
			
		} else {

			ATXsum = 0;
			ATYsum = 0;
			ATXruns = 0;
			ATYruns = 0;
		}

		if(swerve.velocityChecker(3, 0.5)) {
			lockOnTimer += 0.02;
		}

		return new double[] {Double.MAX_VALUE, Double.MAX_VALUE};
	}

	public boolean manipulateCubes(Drivetrain swerve, Limelight limelight, Wrist wrist, WristPositions wristPosition, Claw claw){
		switch(cubeCase){

			case 0:
				cubePosition = lockOnApriltag(swerve, limelight, 1.5);
				if(cubePosition == new double[] {Double.MAX_VALUE, Double.MAX_VALUE}){
					wrist.set(wristPosition);
					cubeCase = 1;
				}
			break;
				
			case 1:
				if(swerve.driveToPosition(cubePosition[0], cubePosition[1], 0.2, 0.05)){
					cubeCase = 2;
				}
			break;

			case 2:

			break;

			case 3:

			break;
		}
	}

}
