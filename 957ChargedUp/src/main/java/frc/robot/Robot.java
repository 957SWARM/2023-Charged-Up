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

public class Robot extends TimedRobot {
	private final XboxController m_controller = new XboxController(0);
	private final Drivetrain m_swerve = new Drivetrain();

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
	final int changeSpeedButton = 1;
	int speedVar = 0;
/* 
	//BUTTONS
	final int highFourBarPosition = 0;
	final int midFourBarPosition = 0;
	final int lowFourBarPosition = 0;
	final int pickupFourBarPosition = 0;
	

	final int deployFourBar = 0;

	final int openClaw = 0;
	final int closeClaw = 0;
*/



	//function for HDC
	public void followTrajectory(double time, Trajectory trajectory){
		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds adjustedSpeeds = controller.calculate(m_swerve.getPose(), goal, Rotation2d.fromDegrees(70.0));
		m_swerve.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
	}
/* 
	public Trajectory getPath(String selectedAuto){
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(selectedAuto);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		 } catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + selectedAuto, ex.getStackTrace());
		 }
		return trajectory;
	}
*/
	@Override
	public void autonomousPeriodic() {
		driveWithJoystick(false);
		m_swerve.updateOdometry();
	}

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
			

		driveWithJoystick(true);

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

		m_swerve.driveAngle(xSpeed, ySpeed, 0, fieldRelative);
	}
}
