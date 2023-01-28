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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private final Drivetrain m_swerve = new Drivetrain();
	private final Limelight limelight = new Limelight();

	//String autoExample = "paths/YourPath.wpilib.json";
	//Trajectory trajectory = new Trajectory();

	HolonomicDriveController controller = new HolonomicDriveController(
		new PIDController(1, 0, 0), new PIDController(1, 0, 0),
		new ProfiledPIDController(1, 0, 0,
		new TrapezoidProfile.Constraints(6.28, 3.14)));
	
	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
	private double speedMult = 1; 
	int speedVar = 0;

	private final SlewRateLimiter m_slewX = new SlewRateLimiter(3);
	private final SlewRateLimiter m_slewY = new SlewRateLimiter(3);
	double angleToHold = 0;
	final int holdAngle = 0;
	boolean holdAngleSwitch = false;

	private static final String kJoystick = "Joystick";
	private static final String kController = "Controller";
	private String driveMode;
	private final SendableChooser<String> driveChooser = new SendableChooser<>();

	private final XboxController m_controller = new XboxController(0);

	//BUTTONS
	// m_controller
	final int changeSpeedButton = 1; //A
	final int highFourBarPosition = 0;
	final int midFourBarPosition = 0;
	final int lowFourBarPosition = 0;
	final int pickupFourBarPosition = 0;	

	//CONTROLLER DRIVE
	int xAxisDrive = 0;
	int yAxisDrive = 1;
	int gTurnAxis = 4;
	int clawToggle = 1;
	int visionCone = 3;
	int visionCube = 4;

	//function for HDC
	public void followTrajectory(double time, Trajectory trajectory){
		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds adjustedSpeeds = controller.calculate(m_swerve.getPose(), goal, Rotation2d.fromDegrees(70.0));
		m_swerve.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
	}

	public Trajectory getPath(String selectedAuto){
		Trajectory trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(selectedAuto);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		 } catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + selectedAuto, ex.getStackTrace());
		 }
		return trajectory;
	}

	@Override
	public void robotInit() {
		driveChooser.setDefaultOption("Controller", kController);
		driveChooser.addOption("Controller", kController);
		driveChooser.addOption("Joystick", kJoystick);
		SmartDashboard.putData("Selected: ", driveChooser);
	}

	@Override
	public void robotPeriodic() {
		
		driveMode = driveChooser.getSelected();
		System.out.println("Selected: " + driveMode);

		switch (driveMode) {
			case kJoystick:
				xAxisDrive = 0;
				yAxisDrive = 1;
				gTurnAxis = 2;
				clawToggle = 1;
				visionCone = 3;
				visionCube = 4;
				break;

			case kController:
				xAxisDrive = 0;
				yAxisDrive = 1;
				gTurnAxis = 4;
				clawToggle = 1;
				visionCone = 3;
				visionCube = 4;
				break;
		}
	}

	@Override
	public void autonomousInit(){

	}

	@Override
	public void autonomousPeriodic() {
		driveWithJoystick(false);
		m_swerve.updateOdometry();
	}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
		switch(speedVar){
			case 0:
			if(m_controller.getRawButtonReleased(changeSpeedButton)){
			speedMult = 1;
			speedVar++;
			}

			case 1: 
			if(m_controller.getRawButtonReleased(changeSpeedButton)){
			speedMult = 0.25;
			speedVar = 0;
			}
		}
		
		if(m_controller.getRawButtonReleased(holdAngle)){
			if(holdAngleSwitch == true){
				holdAngleSwitch = false;
			}
			else if(holdAngleSwitch == false){
				holdAngleSwitch = true;
			}
		}
		
		//driveWithJoystick(true);

		if (m_controller.getRawButton(2)){
			// Getting Distance
			trackAprilTag(0.1, 1.1);

		}else{
			
			driveWithJoystick(true);
		}
	}


	private void driveWithJoystick(boolean fieldRelative) {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed =
			-m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2))
				* Drivetrain.kMaxSpeed * speedMult;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed =
			-m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2))
				* Drivetrain.kMaxSpeed * speedMult;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot =
			-m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.2))
				* Drivetrain.kMaxAngularSpeed;

		//change angleToHold
		double x = m_controller.getRawAxis(4);//x axis
		double y = m_controller.getRawAxis(5);//y axis

		if(x > Math.abs(0.7) || y > Math.abs(0.7) ){
			if(holdAngleSwitch){

				angleToHold = Math.toDegrees(Math.atan(y * (1/x)));
				if(y < 0)
					angleToHold = angleToHold + 180;

				m_swerve.driveAngle(xSpeed, ySpeed, angleToHold, fieldRelative);
			}else{
				m_swerve.driveAngle(xSpeed, ySpeed, 0, fieldRelative);
			}
		}
	}

	public void trackAprilTag(double threshold, double desiredDistance){
		
		double tx = limelight.getAlignmentOffset();
		double distance = limelight.getDistance();
		
		double xSpeed = 0;
		double ySpeed = 0;

		// Distance Adjustment
		if ( distance < desiredDistance - threshold){
			ySpeed = m_slewY.calculate(-1) * 0.5;
		}
		else if ( distance > desiredDistance + threshold){
			ySpeed = m_slewY.calculate(1) * 0.5;
		}
		else{
			ySpeed = m_slewY.calculate(0) * 0.5;
		}

		// Alignment
		if ( tx < - threshold){
			xSpeed = m_slewX.calculate(1) * 0.5;
		}
		else if( tx > threshold) {
			xSpeed = m_slewX.calculate(-1) * 0.5;
		}
		else{
			xSpeed = m_slewX.calculate(0) * 0.5;
		}
		m_swerve.driveAngle(ySpeed, -xSpeed, 0, false);
	}
}

