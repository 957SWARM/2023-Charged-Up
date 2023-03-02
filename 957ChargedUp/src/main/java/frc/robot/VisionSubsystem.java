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
		cubeCase = 0;
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

	public boolean manipulateCubes(Drivetrain swerve, Limelight limelight, Wrist wrist, WristPositions wristPosition, Claw claw, ShooterSpeed shooterSpeed){
		switch(cubeCase){

			case 0:
				System.out.println("Case 0");
				manipulateCubeTimer = 0;
				if(lockOnApriltag(swerve, limelight, .75)){	
					wrist.set(wristPosition);
					cubeCase = 1;
				}
			break;
				
			case 1:
				lockOnTimer = 0;
				if(swerve.driveToPosition(cubePosition[0], 0, 0.2, 0.05)){
					cubeCase = 2;
					claw.clawOuttake(shooterSpeed);
				}
			
				System.out.println(cubePosition[0] + ", " + cubePosition[1]);
			break;

			case 2:
				System.out.println("Case 2");
				if(manipulateCubeTimer > 0.5){
					cubeCase = 0;
					claw.clawStop();
					return true;
				}
				manipulateCubeTimer += 0.02;
			break;
			

		}
		return false;
	}

}
