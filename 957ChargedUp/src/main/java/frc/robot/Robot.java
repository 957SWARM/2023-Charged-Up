// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MiniPID;
import frc.robot.Constants.MoveFourBars;
import frc.robot.Constants.ShooterSpeed;
import frc.robot.Constants.WristPositions;
import frc.robot.Subclasses.ButtonBoardLeft;
import frc.robot.Subclasses.ButtonBoardLeftNew;
import frc.robot.Subclasses.ButtonBoardRight;
import frc.robot.Subclasses.ButtonBoardRightNew;
import frc.robot.Subclasses.DriveController;
import frc.robot.Subclasses.Wing;

public class Robot extends TimedRobot {
	// CLASS CONSTRUCTORS
	private final Drivetrain m_swerve = new Drivetrain();
	private final Limelight m_limelight = new Limelight();
	private final Wrist m_wrist = new Wrist();
	private final FourBar m_fourBar = new FourBar();
	private final Claw m_claw = new Claw();
	private final VisionSubsystem m_vision = new VisionSubsystem();
	private final Shuffleboard m_shuffle = new Shuffleboard();
	private final Bling m_bling = new Bling();
	private final DriveController m_driveController = new DriveController(0);
	private final ButtonBoardLeftNew m_bbLeft = new ButtonBoardLeftNew(1);
	private final ButtonBoardRightNew m_bbRight = new ButtonBoardRightNew(2);
	private final Wing m_wing = new Wing(2, 3);

	//FULL LIST OF ACCESSABLE AUTOS
	String midAutoPart1 = "pathplanner/generatedJSON/midAutoPart1.wpilib.json";
	String midAutoPart2 = "pathplanner/generatedJSON/midAutoPart2.wpilib.json";
	String doubleCubeAuto = "pathplanner/generatedJSON/doubleCubeAuto.wpilib.json";
	Trajectory trajectory = new Trajectory();
	String autoToBeRan;
	double autonomousTimer = 0;
	double teleopTimer = 0;
	double intakeFudge = 0;

	HolonomicDriveController controller = new HolonomicDriveController(
			new PIDController(0.032, 0, 0), new PIDController(-0.000, 0, 0),
			new ProfiledPIDController(1, 0, 0,
					new TrapezoidProfile.Constraints(1.5, 3)));

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
	private double speedMult = 1;
	int speedVar = 0;

	private final SlewRateLimiter m_slewX = new SlewRateLimiter(3);
	private final SlewRateLimiter m_slewY = new SlewRateLimiter(3);
	double angleToHold = 0;

	// CONTROLLERS
	
	//VARS FOR SWITCH STATEMENTS
	int driveVar = 0;
	int trackingVar = 0;
	int clawVar = 0;
	int autoVar = 0;
	int balVar = 0;
	int driveCase = 0;
	int wingVar = 0;

	boolean fourBar = false;
	boolean wrist = false;

	// Shuffleboard
	boolean visionControlled = false;

	// PID
    double autoP = 2.2;
    double autoI = 0;
    double autoD = 0;
    MiniPID autoPID = new MiniPID(autoP, autoI, autoD);
	double yToMaintain;

	Trajectory currentPath;
	Trajectory currentPath2;
	Trajectory currentPath3;

	SlewRateLimiter slew = new SlewRateLimiter(5);

	// function for HDC
	public boolean followTrajectory(double time, Trajectory trajectory) {
		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds adjustedSpeeds = controller.calculate(m_swerve.getPose(), goal, Rotation2d.fromDegrees(0));
		if (time > trajectory.getTotalTimeSeconds()) {
			m_swerve.drive(0, 0, 0, false);
			return true;
		} else {
			m_swerve.autoDrive(adjustedSpeeds);
			return false;
		}
	}

	public Trajectory getPath(String selectedAuto) {
		Trajectory trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(selectedAuto);
			return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + selectedAuto, ex.getStackTrace());
		}
		return trajectory;
	}

	@Override
	public void robotInit() {
		m_wing.retractWing();
		m_bling.timerStart();
		autoPID.setOutputLimits(-.5, .5);
	}

	@Override
	public void robotPeriodic() {
		m_vision.maintainTX(m_limelight);
		m_vision.maintainTY(m_limelight);
		autoToBeRan = m_shuffle.updateAuto();
		m_shuffle.updateShuffleboard(m_swerve, m_claw, m_wrist.getLabel() , m_fourBar.getLabel(), m_vision, speedVar, m_wrist, m_fourBar);
		SmartDashboard.putNumber("roll of bot", m_swerve.m_navx.getRoll());

		if(m_shuffle.takenIntakeFudge >= 0){	// Make sure shuffleboard value is reasonable
			intakeFudge = 0;
		}
		else if(m_shuffle.takenIntakeFudge <= -.4){
			intakeFudge = -.4;
		}
		else{
			intakeFudge = m_shuffle.takenIntakeFudge;
		}

	}

	@Override
	public void autonomousInit() {
		autonomousTimer = 0;
		m_swerve.resetOdometry();

		currentPath = getPath(midAutoPart1);
		currentPath2 = getPath(midAutoPart2);
		currentPath3 = getPath(doubleCubeAuto);
		autoVar = 0;
		if(autoToBeRan != "placeDoNothingCones" && autoToBeRan != "coneLeft" && autoToBeRan != "coneRight" && autoToBeRan != "mid" && autoToBeRan != "cubeMidHigh" && autoToBeRan != "CoolRadicalMidMobility"){
			m_swerve.centerGyro(0);
			m_claw.cubeMode();
		}else{
			m_swerve.centerGyro(.5);
			m_claw.coneMode();
		}
	}

	@Override
	public void autonomousPeriodic() {
		m_swerve.updateOdometry();
		double x = m_swerve.getPose().getX();
		double y = m_swerve.getPose().getY();
		m_fourBar.run();
		m_wrist.run(m_claw.clawPosition(), .2);

		switch(autoToBeRan){
			
			// Old Mid with no mobility
			case "mid":
				m_claw.cubeMode();
				switch(autoVar){
					case 0:
						m_fourBar.setLevel(MoveFourBars.mid);
						m_wrist.set(WristPositions.scoreUp);
						if(autonomousTimer >= 2)
							autoVar++;
					break;

					case 1:
						m_claw.clawOuttake(ShooterSpeed.midCube);
						if(autonomousTimer >= 3.5)
							autoVar++;
					break;

					case 2:
						m_claw.clawStop();
						m_fourBar.setLevel(MoveFourBars.ground);
						m_wrist.set(WristPositions.retract);
						autoBalance();
					break;

					case 3:
					break;
				}
	
			break;

			case "coneLeft":
				switch(autoVar){
					case 0:
						// RAISES ARM and WRIST
						if(autonomousTimer >= 0 && autonomousTimer < 2){
							m_fourBar.setLevel(MoveFourBars.mid);
						// DUNKS CONE
						}else if( autonomousTimer >= 2 && autonomousTimer < 3.5){
							m_wrist.set(WristPositions.scoreOut);
						// DROPS CONE
						}else if( autonomousTimer >= 3.5 && autonomousTimer < 4.5){
							m_claw.cubeMode();
						// RETRACT
						}else if( autonomousTimer >= 4.5 && autonomousTimer < 5){
							m_wrist.set(WristPositions.retract);
						}else if( autonomousTimer >= 5 && autonomousTimer < 7){
							m_fourBar.setLevel(MoveFourBars.ground);
						}else if(autonomousTimer >= 7){
							autonomousTimer = 0;
							autoVar++;
						}	
					break;

					// 
					case 1:

						m_claw.clawStop();
						if (autonomousTimer < .8){
							m_swerve.driveAngle( 2, 0, 180, true, 1);
						}
						else{
							m_swerve.driveAngle(2, autoPID.getOutput(y, 0), 25, true, 2);
							if (m_wrist.getLabel() != "cube ground"){
								m_wrist.set(WristPositions.cubeGround);
							}

						}
						if(x >= 3.6){
							autonomousTimer = 0;
							autoVar ++;
						}
					break;

					case 2:
						m_swerve.driveAngle(0, 0, 25, true, 1);
						if (autonomousTimer > .75){
							autonomousTimer = 0;
							autoVar++;
						}
					break;

					case 3:
						if (autonomousTimer > .5){
							autonomousTimer = 0;
							m_claw.clawIntake(1);
							autoVar++;
						}
					break;

					case 4:
						m_swerve.driveAngle(.5, 0, 25, false, .3);
						m_claw.clawIntake(1);
						if (autonomousTimer > 2){
							m_claw.clawIntake(1);
							m_swerve.drive(0, 0, 0, false);
							// m_wrist.set(WristPositions.cubeGround);
							// m_claw.clawIntake(.5);
						}
					break;
				}		
			break;
		
	
			case "coneRight":
				switch(autoVar){
					case 0:
						if(autonomousTimer == 0)
							m_claw.coneMode();
						if(autonomousTimer >= 0 && autonomousTimer < 3){
							m_fourBar.setLevel(MoveFourBars.mid);
							m_wrist.set(WristPositions.scoreUp);
						}else if( autonomousTimer >= 3 && autonomousTimer < 4){
							m_claw.cubeMode();
						}else if( autonomousTimer >= 4 && autonomousTimer < 7.5){
							m_fourBar.setLevel(MoveFourBars.ground);
							m_wrist.set(WristPositions.retract);
						}else if(autonomousTimer >= 7.5){
							autoVar++;
						}
							
					break;

					case 1:
						m_claw.clawStop();
						m_swerve.driveAngle( -1,0, 0, false, .3);
						if(x <= -3.6){
							autoVar ++;
						}
					break;

					case 2:
						m_swerve.driveAngle( 0,0, 0, false, .3);
					break;
					}
			break;

			case "doNothing":
				switch(autoVar){
					case 0:
					break;
				}
			break;
			

			case "placeDoNothingCube":
				switch(autoVar){
					case 0:
					m_wrist.set(WristPositions.backShootCube);
					if(autonomousTimer >= .5)
						autoVar++;
					break;

					case 1:
						if(autonomousTimer <= 1.5 && autonomousTimer >= .5){
							m_claw.clawStop();
						}else{
							m_claw.clawOuttake(ShooterSpeed.midShootAuto);
						}
					break;
				}
			break;

			case "placeDoNothingCone":
				switch(autoVar){
					case 0:
						if(autonomousTimer == 0)
							m_claw.coneMode();
						if(autonomousTimer >= 0 && autonomousTimer < 3){
							m_fourBar.setLevel(MoveFourBars.mid);
							m_wrist.set(WristPositions.scoreUp);
						}else if( autonomousTimer >= 3 && autonomousTimer < 4){
							m_claw.cubeMode();
						}else if( autonomousTimer >= 4 && autonomousTimer < 7.5){
							m_fourBar.setLevel(MoveFourBars.ground);
							m_wrist.set(WristPositions.retract);
						}
					break;
				}
			break;

			case "crossTheLine": //WORK ON IT IN THE PITS
				switch(autoVar){
					case 0:
						m_swerve.drive(.4, 0, 0, false);
						if(autonomousTimer > 5)
							m_swerve.drive(0, 0, 0, false);
					break;
				}
			break;

			case "cubeMidHigh":
				switch(autoVar){
					case 0:
						if(autonomousTimer == 0)
							m_claw.cubeMode();
						else if(autonomousTimer >= 0 && autonomousTimer < 3){
							m_fourBar.setLevel(MoveFourBars.mid);
							m_wrist.set(WristPositions.midCube);
							m_claw.clawOuttake(ShooterSpeed.midCube);
							}
						else if(autonomousTimer >= 3 && autonomousTimer < 6){
							m_fourBar.setLevel(MoveFourBars.ground);
							m_wrist.set(WristPositions.retract);
							}
					break;
					case 1:
						if(autonomousTimer >= 6 && autonomousTimer < 8){
							//drive
							m_claw.clawIntake(0.5);
						}				
					break;	
					case 2:
						if(autonomousTimer >= 8 && autonomousTimer < 10){
							//place cube high
							m_fourBar.setLevel(MoveFourBars.high);		
							m_wrist.set(WristPositions.highCube);
							m_claw.clawOuttake(ShooterSpeed.highCube);
						}
					break;
				}

			break;
		
			// New mid with MOBILITY!
			case "CoolRadicalMidMobility":
			m_claw.cubeMode();
				switch(autoVar){
					// ARM and WRIST in position
					case 0:
						m_fourBar.setLevel(MoveFourBars.mid);
						m_wrist.set(WristPositions.scoreUp);
						if(autonomousTimer >= 1.75)	//prev 2
							autoVar++;
					break;

					// PLACES Cube
					case 1:
						m_claw.clawOuttake(ShooterSpeed.midCubeSlower);
						if(autonomousTimer >= 3.0)	// prev 3.5
							autoVar++;
					break;

					// RETRACT ARM and WRIST
					case 2:
						m_claw.clawStop();
						m_fourBar.setLevel(MoveFourBars.ground);
						m_wrist.set(WristPositions.retract);
						m_swerve.driveAngle(2, 0, 180, true, 0);
						if (x >= 3.6){
							autonomousTimer = 0;
							autoVar++;
						}
					break;
					
					// TURN AROUND
					case 3:
						m_swerve.driveAngle(0, 0, 0, true, 8);
						if (autonomousTimer > 1){
							autoVar++;
						}
					break;

					// BALANCE
					case 4:
						autoBalance();
					break;
					}
			}

		autonomousTimer += 0.02;
		
	}

			
				
				
	public void autoBalance(){
		double modPitch = m_swerve.m_navx.getRoll();
		switch(balVar){
			case 0: // hits
				m_swerve.drive(-1, 0, 0, false);
				if(modPitch < -10)
					balVar++;
			break;

			case 1: //half on
				m_swerve.drive(-.6, 0, 0, false);
				if(modPitch > -8)
					balVar++;
			break;

			case 2: // slight balanced
				if(modPitch < -2){
					m_swerve.drive(-.25, 0, 0, false);
				}else if(modPitch > 2){
					m_swerve.drive(.25, 0, 0, false);
				}else{
					m_swerve.drive(0, 0, 0, false);
				}
			break;

		}
	}

	@Override
	public void teleopInit() {
		teleopTimer = 0;
	}

	@Override
	public void teleopPeriodic() {
		teleopTimer += .02;
		
		m_swerve.updateOdometry();
		if (m_driveController.shifter()){ // sets to fast speed: 4.2 m/s
			speedMult = 1;
			speedVar = 0;
		}
		speedShift(m_driveController.slowMidSpeed());

		switch(driveCase){

			// Normal Driving
			case 0:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
						-m_driveController.getRightStickX());
				m_vision.resetCases();
				if(m_bbRight.vision1()){
					driveCase = 3;
				}
				else if(m_bbRight.vision2()){
					driveCase = 1;
				}
				else if(m_bbRight.vision3()){
					driveCase = 2;
				
				}
			break;

			// Normal Driving but waiting for Cube button Unpress
			case 1:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
					-m_driveController.getRightStickX());
				if(!m_bbRight.vision2()){
					driveCase = 4;
				}

			break;

			// Normal Driving but waiting for Cone right button Unpress
			case 2:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
					-m_driveController.getRightStickX());
				if(!m_bbRight.vision3()){
					driveCase = 5;
				}
			
			// Normal Driving but waiting for Cone left button unpress
			case 3:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
					-m_driveController.getRightStickX());
				if(!m_bbRight.vision1()){
					driveCase = 6;
				}

			// Cube
			case 4:
				// System.out.println("Case 4");
				visionControlled = true;
				if(m_bbRight.vision2()){
					driveCase = 7;
				}
				if(m_vision.positionFromAT(m_swerve, m_limelight, 0))
					driveCase = 7;
			break;

			// Cone right
			case 5:
				// System.out.println("Case 5");
				visionControlled = true;
				if(m_bbRight.vision3()){
					driveCase = 8;
				}
				if(m_vision.positionFromAT(m_swerve, m_limelight, .56))
					driveCase = 8;
			break;

			// Cone left
			case 6:
				// System.out.println("Case 6");
				visionControlled = true;
				if(m_bbRight.vision1()){
					driveCase = 9;
				}
				if(m_vision.positionFromAT(m_swerve, m_limelight, -.56))
					driveCase = 9;
			break;

			// Cancel Case (Cubes)
			case 7:
				visionControlled = false;
				if(!m_bbRight.vision2()){
					driveCase = 0;
				}

			break;

			// Cancel Case (Cone right)
			case 8:
				m_limelight.setPipe(0);
				visionControlled = false;
				if(!m_bbRight.vision3()){
					driveCase = 0;
				}
			
			// Cancel Case (Cone left)
			case 9:
				m_limelight.setPipe(0);
				visionControlled = false;
				if(!m_bbRight.vision1()){
					driveCase = 0;
			}
		
		}

		if (m_driveController.clawIntake() > .5) { 
			m_claw.clawIntake(1 + intakeFudge); // intakeFudge Change
		} else if (m_driveController.highCube()) { 
			m_claw.clawOuttake(ShooterSpeed.highCube);
		} else if (m_driveController.midCube()) { 
			m_claw.clawOuttake(ShooterSpeed.midCube);
		} else if (m_driveController.place()){
			m_claw.clawOuttake(ShooterSpeed.placeCube);
		}else if (m_driveController.clawIntakeStop() > .5 || m_claw.m_limitSwitch.get() == false) { // right trigger
			m_claw.clawStop();
		}else if(m_driveController.centerGyro()){
			m_swerve.centerGyro(0);
		}
		// System.out.println("Case " + wingVar);

		fourBar = !m_fourBar.m_limitSwitch.get();
		wrist = m_wrist.m_limitSwitch.get();

		switch(wingVar){
			
			case 0:
				m_wing.retractWing();
				if(m_driveController.wingToggle())
					wingVar++;
			break;

			case 1:
				m_wing.retractWing();
				if(!m_driveController.wingToggle()){
					wingVar++;
					
				}
			break;

			case 2:
				if (fourBar && wrist){
					m_wing.extendWing();
				}else{
					wingVar = 0;
				}

				if(m_driveController.wingToggle())
					wingVar++;
			break;

			case 3:
				if (!fourBar || !wrist){
					wingVar = 0;
				}
				if(!m_driveController.wingToggle()){
					wingVar = 0;
				}
			break;

		}
		 // System.out.println("four bar: " + fourBar);
		 // System.out.println("wrist: " + wrist);

		switch (clawVar) {
			case 0:
				m_claw.coneMode();
				if (m_driveController.clawToggle())
					clawVar = 1;
				break;

			case 1:
				m_claw.coneMode();
				if (!m_driveController.clawToggle())
					clawVar = 2;
				break;

			case 2:
				m_claw.cubeMode();
				if (m_driveController.clawToggle())
					clawVar = 3;
				break;

			case 3:
				m_claw.cubeMode();
				if (!m_driveController.clawToggle())
					clawVar = 0;
				break;
			}
	
		/*
		//CURRENT OLD BUTTON BOARD CONTROLLS
		if (m_bbRight.wristRetract())
			m_wrist.set(WristPositions.retract);
		if (m_bbRight.wrist45())
			m_wrist.set(WristPositions.scoreUp);
		if (m_bbRight.wristOut())
			m_wrist.set(WristPositions.scoreOut);
		if(m_bbRight.wristSub())
			m_wrist.set(WristPositions.substation);	
		if (m_bbRight.vision3())
			m_wrist.set(WristPositions.backShootCube);


		if(m_bbLeft.pickupCone())
			m_wrist.set(WristPositions.coneGround);
		if(m_bbRight.pickupCube())
			m_wrist.set(WristPositions.cubeGround);
		if(m_bbRight.wristSub())
			m_wrist.set(WristPositions.scoreSub);

		m_wrist.run(m_claw.clawPosition(), 0.3);

		if (m_bbLeft.armMax())
			m_fourBar.setLevel(MoveFourBars.high);
		if (m_bbLeft.armMid())
			m_fourBar.setLevel(MoveFourBars.mid);
		if (m_bbLeft.armSub())
			m_fourBar.setLevel(MoveFourBars.substation);
		if (m_bbLeft.armRectract())
			m_fourBar.setLevel(MoveFourBars.ground);
		
		if(m_bbLeft.blingCone()){
			m_bling.blingSend(1);
		}
		if(m_bbLeft.blingCube()){
			m_bling.blingSend(2);
		}
	*/

	// NEW BUTTON BOARD CONTROLSSSSSSSSSSSSSSSSS
	if(m_bbLeft.blingCone())
		m_bling.blingSend(1);
	if(m_bbLeft.blingCube())
		m_bling.blingSend(2);
	m_bling.blingRan();

	if(m_bbLeft.wristOut())
		m_wrist.set(WristPositions.scoreOut);
	if(m_bbLeft.wrist45())
		m_wrist.set(WristPositions.scoreUp);
	
	if(m_bbLeft.retractArm())
		m_fourBar.setLevel(MoveFourBars.ground);
	if(m_bbLeft.retractWrist())
		m_wrist.set(WristPositions.retract);

	if(m_bbLeft.conePickup())
		m_wrist.set(WristPositions.coneGround);
	if(m_bbLeft.singleSub()){
		m_fourBar.setLevel(MoveFourBars.ground);
		m_wrist.set(WristPositions.scoreUp);
	}

	if(m_bbRight.highCube()){
		m_fourBar.setLevel(MoveFourBars.high);
		m_wrist.set(WristPositions.scoreUp);
	}
	if(m_bbRight.midCube()){
		m_fourBar.setLevel(MoveFourBars.mid);
		m_wrist.set(WristPositions.scoreOut);
	}
	
	if(m_bbRight.fourBarMid()){
		m_fourBar.setLevel(MoveFourBars.mid);
	}
	if(m_bbRight.doubleSubstation()){
		m_fourBar.setLevel(MoveFourBars.mid);
		m_wrist.set(WristPositions.scoreSub);
	}

	if(m_bbRight.cubePickup()){
		m_wrist.set(WristPositions.cubeGround);

	}


	// Extra. Runs ARM, BLING, and Rumble 

		m_fourBar.run();
		m_wrist.run(m_claw.clawPosition(), .3);


		if(teleopTimer >= 95 && teleopTimer <= 97){	
			m_driveController.setRumble(RumbleType.kBothRumble, 1);
		}else{
			m_driveController.setRumble(RumbleType.kBothRumble, 0);
		}

	}

	public void speedShift(boolean getRawButtonReleased) {

		switch (speedVar) {
			case 0:
				if (getRawButtonReleased) {
					speedVar++;
				}
				break;

			case 1:
				speedMult = 0.6;
				if (!getRawButtonReleased) {
					speedVar++;
				}
				break;

			case 2:
				if (getRawButtonReleased) {
					speedVar++;
				}
				break;

			case 3:
				speedMult = .3;
				if (!getRawButtonReleased) {
					speedVar = 0;
				}
				break;
		}
	}

	public void driveAngle(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

		double[] arr = deadZone(xSpeed, ySpeed, 0.2);
		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband( xSpeed, 0.05));
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband( ySpeed, 0.05));

		// Check if xSpeed or ySpeed are past max (speedMult), if so set to max
		if (xSpeed > speedMult){
			xSpeed = speedMult;
		}
		else if (xSpeed < -speedMult){
			xSpeed = -speedMult;
		}
		if (ySpeed > speedMult){
			ySpeed = speedMult;
		}
		else if (ySpeed < -speedMult){
			ySpeed = -speedMult;
		}
		m_swerve.driveAngle(-ySpeed * 4.2, -xSpeed * 4.2, rot, fieldRelative, .3);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {


		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband( xSpeed, 0.05));
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband( ySpeed, 0.05));
		rot = m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.2));
		// Check if xSpeed or ySpeed are past max (speedMult), if so set to max
		if (xSpeed > speedMult){
			xSpeed = speedMult;
		}
		else if (xSpeed < -speedMult){
			xSpeed = -speedMult;
		}
		if (ySpeed > speedMult){
			ySpeed = speedMult;
		}
		else if (ySpeed < -speedMult){
			ySpeed = -speedMult;
		}
		m_swerve.drive(-ySpeed * 4.2, -xSpeed * 4.2, rot * 6.28 * speedMult, fieldRelative);
	}

	public double joyAngle(double x, double y) {
		if (0.7 < Math.abs(x) || 0.7 < Math.abs(y)) {
			angleToHold = Math.toDegrees(Math.atan2(x, -y));
		}else{
			return angleToHold;
		}

		return angleToHold;
	}

	public static double[] deadZone(double xAxis, double yAxis, double deadZoneRange) {
		double deadZoneArray[];
		deadZoneArray = new double[2];
		if (Math.abs(xAxis) < deadZoneRange && Math.abs(yAxis) < deadZoneRange) {
			xAxis = 0;
			yAxis = 0;
		}
		deadZoneArray[0] = xAxis;
		deadZoneArray[1] = yAxis;
		return deadZoneArray;
	}

	public void driveMode(double x, double y, double rot) {
		switch (driveVar) {
			case 0:
				drive(x, y, rot, true);
				if (m_driveController.driveStyle())
					driveVar++;
				break;

			case 1:
				drive(x, y, rot, true);
				if (!m_driveController.driveStyle())
					driveVar++;
				break;

			case 2:
				driveAngle(x, y, joyAngle(Math.round(m_driveController.getRightStickX()), Math.round(m_driveController.getRightStickY())), true);
				if (m_driveController.driveStyle())
					driveVar++;
				break;

			case 3:
				driveAngle(x, y, joyAngle(Math.round(m_driveController.getRightStickX()), m_driveController.getRightStickY()), true);
				if (!m_driveController.driveStyle())
					driveVar = 0;
				break;
		}
	}

	@Override
	public void testInit(){

	}

	@Override 
	public void testPeriodic(){
		System.out.println(m_claw.clawPosition());
	}
}
