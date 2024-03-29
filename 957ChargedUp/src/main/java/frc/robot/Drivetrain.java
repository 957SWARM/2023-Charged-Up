	// Copyright (c) FIRST and other WPILib contributors.
	// Open Source Software; you can modify and/or share it under the terms of
	// the WPILib BSD license file in the root directory of this project.

	package frc.robot;


	import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Constants.MiniPID;



	/** Represents a swerve drive style drivetrain. */
	public class Drivetrain {
	public static final double kMaxSpeed = 4.2; // 3 meters per second
	public static final double kMaxAngularSpeed = Math.PI * 2 ; // 1 rotation per second

	private final Translation2d m_frontLeftLocation = new Translation2d(0.3302, 0.3302); //13 inches converted into meters through google
	private final Translation2d m_frontRightLocation = new Translation2d(0.3302, -0.3302);
	private final Translation2d m_backLeftLocation = new Translation2d(-0.3302, 0.3302);
	private final Translation2d m_backRightLocation = new Translation2d(-0.3302, -0.3302);

	private final MAXSwerveModule m_frontRight = new MAXSwerveModule(1, 2, (1-0.305) * 6.28);	// .
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(3, 4, (1-0.518-0.25) * 6.28);	// .
	private final MAXSwerveModule m_backLeft = new MAXSwerveModule(5, 6, (1-0.977+0.5) * 6.28);	//0.
	private final MAXSwerveModule m_backRight = new MAXSwerveModule(7, 8, (1-0.742+0.25) * 6.28);	//.

	AHRS m_navx = new AHRS(Port.kMXP);
	PIDController tapePID = new PIDController(0.1, 0, 0);

	PIDController ATXPID = new PIDController(1, 0, 0);
	PIDController ATYPID = new PIDController(3, 0, 0);
	double frontRightAngle;

    double v_vkp = 0.1;
    double v_vki = 0.;
    double v_vkd = 0;
    MiniPID visionPID = new MiniPID(v_vkp,v_vki,v_vki);

	double angleOffset = 0;
	double xPosition = 0;

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
    	visionPID.setOutputLimits(-6.0, 6.0);
	}



	/**
	* Method to drive the robot using joystick info.
	*
	* @param xSpeed Speed of the robot in the x direction (forward).
	* @param ySpeed Speed of the robot in the y direction (sideways).
	* @param rot Angular rate of the robot.
	* @param fieldRelative Whether the provided x and y speeds are relative to the field.
	*/



	public void driveAngle(double xSpeed, double ySpeed, double angle, boolean fieldRelative, double maxRotSpeed){
		double pidAngle = -visionPID.getOutput((m_navx.getAngle() + angleOffset * 360), angle);
		if (pidAngle >= maxRotSpeed){
			pidAngle = maxRotSpeed;
		}
		else if (pidAngle <= -maxRotSpeed){
			pidAngle = -maxRotSpeed;
		}
		var swerveModuleStates =
			m_kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, pidAngle,  m_navx.getRotation2d().minus(new Rotation2d(6.28 * angleOffset)))
					: new ChassisSpeeds(xSpeed, ySpeed, pidAngle));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

	}

	public double getFrontLeftSwerveModuleAngle(){
		return frontRightAngle = m_frontLeft.getPosition().angle.getDegrees();
	}
	public double getFrontRightSwerveModuleAngle(){
		return frontRightAngle = m_frontRight.getPosition().angle.getDegrees();
	}
	public double getBackLeftSwerveModuleAngle(){
		return frontRightAngle = m_backLeft.getPosition().angle.getDegrees();
	}
	public double getBackRightSwerveModuleAngle(){
		return frontRightAngle = m_backRight.getPosition().angle.getDegrees();
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates =
			m_kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navx.getRotation2d().minus(new Rotation2d(6.28 * angleOffset)))
					: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

	} 


	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		Rotation2d calculatedAngle = m_navx.getRotation2d().minus(new Rotation2d(3.14));
		
		m_odometry.update(
			calculatedAngle,
			new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
			});
	}

	public void resetOdometry() {
		m_odometry.resetPosition(
			m_navx.getRotation2d(), 			
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_backLeft.getPosition(),
				m_backRight.getPosition()},
			new Pose2d());	
	}

	public void centerGyro(double offset){
		angleOffset = m_navx.getRotation2d().getRotations() + offset;
	}

	public Pose2d getPose(){
		double x_pos = m_odometry.getPoseMeters().getX();
		double y_pos = m_odometry.getPoseMeters().getY();
		Rotation2d rot = m_odometry.getPoseMeters().getRotation();
		return m_odometry.getPoseMeters();
	}

	public void autoDrive(ChassisSpeeds chassisSpeeds){
		SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
	}
	// driveMeters for april tags
	/* 
	public boolean realestDriveMeters(double meters, double speed, double threshold){
		double realestMeters = 0;
		if (meters >= 0){
			drive(0, speed, 0, false);
			realestMeters = meters - .1;
		}
		else{
			drive(0, -speed, 0, false);
			realestMeters = meters;
		}
		double xposition = m_odometry.getPoseMeters().getY();
		//System.out.println(Math.abs(meters) - Math.abs(xposition));
		if (Math.abs(realestMeters) - Math.abs(xposition) < threshold){
			drive(0, 0, 0, false);
			return true;
		}
		return false;
	}
	*/

	// driveMeters for tape tracking
	public boolean loserDriveMeters(double meters, double speed, double threshold){
		if (meters >= 0){
			drive(0, speed, 0, false);
		}
		else{
			drive(0, -speed, 0, false);
		}
		double xposition = m_odometry.getPoseMeters().getY();
		//System.out.println(Math.abs(meters) - Math.abs(xposition));
		if (Math.abs(meters) - Math.abs(xposition) < threshold){
			drive(0, 0, 0, false);
			return true;
		}
		return false;
	}

	public boolean velocityChecker(double n, double s){
		double frv = Math.abs(m_frontRight.getVelocity());
		double flv = Math.abs(m_frontLeft.getVelocity());
		double brv = Math.abs(m_backRight.getVelocity());
		double blv = Math.abs(m_backLeft.getVelocity());

		boolean isBelow = false;

		if(frv < s/n){
			isBelow = true;
		}
		if(flv < s/n){
			isBelow = true;
		}
		if(brv < s/n){
			isBelow = true;
		}
		if(blv < s/n){
			isBelow = true;
		}
		return isBelow;
	}


	//Drive to position
    public boolean driveToPosition(double targetPositionX, double targetPositionY, double maxSpeed, double threshold){
        double motorOutputX = ATXPID.calculate(m_odometry.getPoseMeters().getX(), targetPositionX);
        double motorOutputY = ATYPID.calculate(-m_odometry.getPoseMeters().getY(), targetPositionY - .06);

        if(motorOutputX > maxSpeed) {
            motorOutputX = maxSpeed;
        }
        if(motorOutputX < -maxSpeed) {
            motorOutputX = -maxSpeed;
        }
        if(motorOutputY > maxSpeed) {
            motorOutputY = maxSpeed;
        }
        if(motorOutputY < -maxSpeed) {
            motorOutputY = -maxSpeed;
        }

        boolean x_condition = Math.abs(targetPositionX - m_odometry.getPoseMeters().getX()) < threshold;
        boolean y_condition = Math.abs(targetPositionY - m_odometry.getPoseMeters().getY()) < threshold;

        if(y_condition) {
            drive(0, 0, 0, false);
            return true;
        }

        drive(0, motorOutputY, 0, false);
        xPosition = m_odometry.getPoseMeters().getY();
        // System.out.println("Y is: " + m_odometry.getPoseMeters().getY());

        return false;
    }
	public boolean trackTapePID(double currentPosition, double maxSpeed, double threshold){
		double motorOutput = tapePID.calculate(currentPosition, -3);

		if(motorOutput > maxSpeed) {
			motorOutput = maxSpeed;
		}
		if(motorOutput < -maxSpeed) {
			motorOutput = -maxSpeed;
		}

		if(currentPosition < threshold && currentPosition > -threshold) {
			drive(0, 0, 0, false);
			return true;
		}

		driveAngle(0, motorOutput, 180, false, .3);	
		return false;
	}

		// Hit wall variables (move to drive train)
		int wallstate = 0;
		double timer = 0;
		double length = 0.5;
		double speed = 0.7;
		double n = 2;
	
		public boolean hitwall(){
	
		switch(wallstate){
			case 0:
	
				driveAngle(speed, 0, 0, false, .3);
				timer += 0.02;
				if(timer > length){
					wallstate = 1;
					return false;
				}
	
			  break;
			case 1:
			
				driveAngle(speed, 0, 0, false, .3);
				timer = 0;
				if(velocityChecker(n, speed)){
					wallstate = 0;
					return true;
				}
	
			  break;
	
			}
			return false;
		}
}
