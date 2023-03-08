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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MoveFourBars;
import frc.robot.Constants.ShooterSpeed;
import frc.robot.Constants.WristPositions;
import frc.robot.Subclasses.ButtonBoardLeft;
import frc.robot.Subclasses.ButtonBoardRight;
import frc.robot.Subclasses.DriveController;

public class Robot extends TimedRobot {
	// CLASS CONSTRUCTORS
	private final Drivetrain m_swerve = new Drivetrain();
	private final Limelight m_limelight = new Limelight();
	private final Wrist m_wrist = new Wrist();
	private final FourBar m_fourBar = new FourBar();
	private final Claw m_claw = new Claw();
	private final VisionSubsystem m_vision = new VisionSubsystem();
	// private final Bling m_bling = new Bling();

	//FULL LIST OF ACCESSABLE AUTOS
	String midAutoPart1 = "pathplanner/generatedJSON/midAutoPart1.wpilib.json";
	String midAutoPart2 = "pathplanner/generatedJSON/midAutoPart2.wpilib.json";
	Trajectory trajectory = new Trajectory();
	String autoToBeRan = "test2";
	double autonomousTimer = 0;
	double teleopTimer = 0;

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
	private final DriveController m_driveController = new DriveController(0);
	private final ButtonBoardLeft m_bbLeft = new ButtonBoardLeft(1);
	private final ButtonBoardRight m_bbRight = new ButtonBoardRight(2);

	//VARS FOR SWITCH STATEMENTS
	int driveVar = 0;
	int trackingVar = 0;
	int clawVar = 0;
	int autoVar = 0;
	int balVar = 0;
	int driveCase = 0;

	// SHUFFLEBOARD VAR
	boolean visionControlled = false;

	Trajectory currentPath;
	Trajectory currentPath2;

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
		// m_bling.timerStart();
	}

	@Override
	public void robotPeriodic() {
		m_vision.maintainTX(m_limelight);
		m_vision.maintainTY(m_limelight);

		Shuffleboard.updateShuffleboard(m_swerve, m_claw, m_wrist.getLabel() , m_fourBar.getLabel(), m_vision, speedVar);
		SmartDashboard.putNumber("roll of bot", m_swerve.m_navx.getRoll());
	}

	@Override
	public void autonomousInit() {
		autonomousTimer = 0;
		m_swerve.resetOdometry();

		currentPath = getPath(midAutoPart1);
		currentPath2 = getPath(midAutoPart2);
		autoVar = 0;
		autoToBeRan = "test2";
		if(autoToBeRan == "test")
			m_claw.cubeMode();
	}

	@Override
	public void autonomousPeriodic() {
		m_swerve.updateOdometry();
		double x = m_swerve.getPose().getX();
		double y = m_swerve.getPose().getY();
		m_fourBar.run();
		m_wrist.run();

		switch(autoToBeRan){
			
			case "mid":
				switch(autoVar){
					case 0:
						m_wrist.set(WristPositions.backShootCube);
						if(autonomousTimer <= .5){
							m_claw.clawStop();
						}else{
							m_claw.clawOuttake(ShooterSpeed.midShootAuto);
						}
						if(autonomousTimer >= 1)
							autoVar++;
					break;

					case 1:
						m_claw.clawStop();
						m_swerve.driveAngle( 1.5,0, 0, true);
						if(x >= 4)
							autoVar ++;
					break;

					case 2:
						autoBalance();
					break;
				}
			break;

			case "coneLeft":
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
						m_swerve.driveAngle( -1,0, 0, true);
						if(x <= -3.6){
							autoVar ++;
						}
					break;

					case 2:
						m_swerve.driveAngle( 0,0, 0, true);
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
						m_swerve.driveAngle( -1,0, 0, true);
						if(x <= -3.6){
							autoVar ++;
						}
					break;

					case 2:
						m_swerve.driveAngle( 0,0, 0, true);
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
						if(autonomousTimer <= .5){
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
		}
		autonomousTimer += 0.02;
	}

	public void autoBalance(){
		double modPitch = m_swerve.m_navx.getRoll();
		switch(balVar){
			case 0: // hits
				m_swerve.drive(-1, 0, -0, true);
				if(modPitch < -10)
					balVar++;
			break;

			case 1: //half on
				m_swerve.drive(-.6, 0, 0, true);
				if(modPitch > -8)
					balVar++;
			break;

			case 2: // slght balanced
				if(modPitch > 2){
					m_swerve.drive(.2, 0, 0, true);
				}else if(modPitch < -2){
					m_swerve.drive(-.2, 0, 0, true);
				}else{
					m_swerve.drive(0, 0, 0, true);
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

		m_swerve.updateOdometry();
		if (m_driveController.slowMidSpeed()) // sets to fast speed: 4.2 m/s
			speedMult = 1;
		speedShift(m_driveController.shifter());

		switch(driveCase){

			// Normal Driving
			case 0:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
						-m_driveController.getRightStickX());
				m_vision.resetCases();
				if(m_bbRight.vision1()){
					driveCase = 1;
				}
				else if(m_bbRight.vision2()){
					driveCase = 2;
				}
			break;

			// Normal Driving but waiting for Cube button Unpress
			case 1:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
					-m_driveController.getRightStickX());
				if(!m_bbRight.vision1()){
					driveCase = 3;
				}

			break;

			// Normal Driving but waiting for Cone button Unpress
			case 2:
				visionControlled = false;
				driveMode(m_driveController.getLeftStickX(), m_driveController.getLeftStickY(),
					-m_driveController.getRightStickX());
				if(!m_bbRight.vision2()){
					driveCase = 4;
				}

			// Cube
			case 3:
				System.out.println("Cube");
				visionControlled = true;
				if(m_bbRight.vision1()){
					driveCase = 5;
				}
				if(m_vision.manipulateCubes(m_wrist.wristPos, m_swerve, m_limelight, m_wrist))
					driveCase = 5;
			break;

			// Cone
			case 4:
				System.out.println("Cone");
				visionControlled = true;
				if(m_bbRight.vision2()){
					driveCase = 6;
				}
				if(m_vision.TapeTracking(m_wrist.wristPos, m_swerve, m_limelight, m_wrist))
					driveCase = 6;
			break;

			// Cancel Case (Cubes)
			case 5:
				visionControlled = false;
				if(!m_bbRight.vision1()){
					driveCase = 0;
				}

			break;

			// Cancel Case (Cones)
			case 6:
				m_limelight.setPipe(0);
				visionControlled = false;
				if(!m_bbRight.vision2()){
					driveCase = 0;
				}
		}

		if (m_driveController.clawIntake() > .5) { 
			m_claw.clawIntake(.5);
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

		m_wrist.run();

		if (m_bbLeft.armMax())
			m_fourBar.setLevel(MoveFourBars.high);
		if (m_bbLeft.armMid())
			m_fourBar.setLevel(MoveFourBars.mid);
		if (m_bbLeft.armSub())
			m_fourBar.setLevel(MoveFourBars.substation);
		if (m_bbLeft.armRectract())
			m_fourBar.setLevel(MoveFourBars.ground);
		/*
		if(m_bbLeft.blingCone()){
			m_bling.blingSend(1);
		}
		if(m_bbLeft.blingCube()){
			m_bling.blingSend(2);
		}
		*/

		m_fourBar.run();

		if(teleopTimer >= 105 && teleopTimer <= 107){	
			m_driveController.setRumble(RumbleType.kBothRumble, 1);
		}else{
			m_driveController.setRumble(RumbleType.kBothRumble, 0);
		}

	}

	public void speedShift(boolean getRawButtonReleased) {
		

		switch (speedVar) {
			case 0:
				speedMult = 0.6;
				if (m_driveController.shifter()) {
					speedVar++;
				}
				break;

			case 1:
				speedMult = 0.6;
				if (!m_driveController.shifter()) {
					speedVar++;
				}
				break;

			case 2:
				speedMult = 0.36;
				if (m_driveController.shifter()) {
					speedVar++;
				}
				break;

			case 3:
				speedMult = 0.36;
				if (!m_driveController.shifter()) {
					speedVar = 0;
				}
				break;
		}
	}

	public void driveAngle(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

		double[] arr = deadZone(xSpeed, ySpeed, 0.2);
		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.05));
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.05));

		m_swerve.driveAngle(-ySpeed * speedMult * 4.2, -xSpeed * speedMult * 4.2, rot, fieldRelative);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {


		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.05));
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.05));
		rot = m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.2));
		m_swerve.drive(-ySpeed * speedMult * 4.2, -xSpeed * speedMult * 4.2, rot * 6.28 * speedMult, fieldRelative);
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
		//test for rumble
		if(DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() >= 25){
			m_driveController.setRumble(RumbleType.kBothRumble, 1);
		}
	}
}
