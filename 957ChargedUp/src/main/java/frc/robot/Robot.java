// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
	private final XboxController m_controller = new XboxController(0);
	private final Drivetrain m_swerve = new Drivetrain();

	HolonomicDriveController controller = new HolonomicDriveController(
	new PIDController(1, 0, 0), new PIDController(1, 0, 0),
	new ProfiledPIDController(1, 0, 0,
		new TrapezoidProfile.Constraints(6.28, 3.14)));
	
	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

	//function for HDC
	public void holonomicDrive(double time, Trajectory trajectory){
		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds adjustedSpeeds = controller.calculate(trajectory, goal, Rotation2d.fromDegrees(70.0));
	}


	@Override
	public void autonomousPeriodic() {
		driveWithJoystick(false);
		m_swerve.updateOdometry();
	}

	@Override
	public void teleopPeriodic() {
		driveWithJoystick(true);
	}

	private void driveWithJoystick(boolean fieldRelative) {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed =
			-m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
				* Drivetrain.kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed =
			-m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
				* Drivetrain.kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot =
			-m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
				* Drivetrain.kMaxAngularSpeed;

		m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
	}
}
