	// Copyright (c) FIRST and other WPILib contributors.
	// Open Source Software; you can modify and/or share it under the terms of
	// the WPILib BSD license file in the root directory of this project.

	package frc.robot;


	import com.kauailabs.navx.frc.AHRS;

	//import com.kauailabs.navx.frc.AHRS;

	import edu.wpi.first.math.geometry.Pose2d;
	import edu.wpi.first.math.geometry.Translation2d;
	import edu.wpi.first.math.kinematics.ChassisSpeeds;
	import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
	import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
	import edu.wpi.first.math.kinematics.SwerveModulePosition;
	import edu.wpi.first.wpilibj.SPI.Port;



	/** Represents a swerve drive style drivetrain. */
	public class Drivetrain {
	public static final double kMaxSpeed = 3.0; // 3 meters per second
	public static final double kMaxAngularSpeed = Math.PI ; // 1/2 rotation per second

	private final Translation2d m_frontLeftLocation = new Translation2d(0.3302, 0.3302); //13 inches converted into meters through google
	private final Translation2d m_frontRightLocation = new Translation2d(0.3302, -0.3302);
	private final Translation2d m_backLeftLocation = new Translation2d(-0.3302, 0.3302);
	private final Translation2d m_backRightLocation = new Translation2d(-0.3302, -0.3302);

	private final MAXSwerveModule m_frontRight = new MAXSwerveModule(1, 2, (1-0.839) * 6.28);
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(3, 4, (1-0.556-0.25) * 6.28);
	private final MAXSwerveModule m_backLeft = new MAXSwerveModule(5, 6, (1-0.138+0.5) * 6.28);
	private final MAXSwerveModule m_backRight = new MAXSwerveModule(7, 8, (1-0.893+0.25) * 6.28);

	AHRS m_navx = new AHRS(Port.kMXP);

    double v_vkp = 0.1;
    double v_vki = 0.;
    double v_vkd = 0;
    MiniPID visionPID = new MiniPID(v_vkp,v_vki,v_vki);

	private final SwerveDriveKinematics m_kinematics =
		new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	private final SwerveDriveOdometry m_odometry =
		new SwerveDriveOdometry(
			m_kinematics,
			m_navx.getRotation2d(),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_backLeft.getPosition(),
				m_backRight.getPosition()
			});

	public Drivetrain() {
	  	m_navx.reset();
    	visionPID.setOutputLimits(-2.0, 2.0);
	}

	/**
	* Method to drive the robot using joystick info.
	*
	* @param xSpeed Speed of the robot in the x direction (forward).
	* @param ySpeed Speed of the robot in the y direction (sideways).
	* @param rot Angular rate of the robot.
	* @param fieldRelative Whether the provided x and y speeds are relative to the field.
	*/

	public void driveAngle(double xSpeed, double ySpeed, double angle, boolean fieldRelative){
		double pidAngle = -visionPID.getOutput(m_navx.getAngle(), angle);

		var swerveModuleStates =
			m_kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, pidAngle, m_navx.getRotation2d())
					: new ChassisSpeeds(xSpeed, ySpeed, pidAngle));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

		//System.out.println(swerveModuleStates[0].angle);
	}


	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates =
			m_kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navx.getRotation2d())
					: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

		//System.out.println(swerveModuleStates[0].angle);
	} 

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(
			m_navx.getRotation2d(),
			new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
			});
	}

	public Pose2d getPose(){
		return m_odometry.getPoseMeters();
	}

}
