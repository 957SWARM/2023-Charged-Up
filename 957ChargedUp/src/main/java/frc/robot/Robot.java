// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ResourceBundle.Control;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerButtons;
import frc.robot.Constants.JoystickButtons;

public class Robot extends TimedRobot {
	private final Drivetrain m_swerve = new Drivetrain();
	private final Limelight limelight = new Limelight();

	String autoExample = "pathplanner/generatedJSON/TestPath.wpilib.json";
	Trajectory trajectory = new Trajectory();
	double autonomousTimer = 0;

	HolonomicDriveController controller = new HolonomicDriveController(
		new PIDController(0, 0, 0), new PIDController(0, 0, 0),
		new ProfiledPIDController(0, 0, 0,
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
	final int holdAngle = 0;
	boolean holdAngleSwitch = false;

	private final Joystick m_controller = new Joystick(0);

	private static final String kJoystick = "Joystick";
	private static final String kController = "Controller";
	private String driveMode;
	private final SendableChooser<String> driveChooser = new SendableChooser<>();

	//BUTTONS
	// m_controller
	final int changeSpeedButton = 1; //A
	final int highFourBarPosition = 0;
	final int midFourBarPosition = 0;
	final int lowFourBarPosition = 0;
	final int pickupFourBarPosition = 0;	


	//CONTROLLER DRIVE
	int xAxisDrive = 0;
	int yAxisDrive = 0;
	int gTurnAxis = 0;
	int clawToggle = 0;
	int visionCone = 0;
	int visionCube = 0;
	int shifter = 0;
	int driveStyle = 0;

	int var = 0;

	Trajectory currentPath;

	//function for HDC
	public void followTrajectory(double time, Trajectory trajectory){
		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds adjustedSpeeds = controller.calculate(m_swerve.getPose(), goal, Rotation2d.fromDegrees(70.0));
		m_swerve.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, false);
	}

	public Trajectory getPath(String selectedAuto){
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
		driveChooser.setDefaultOption("Joystick", kJoystick);
		driveChooser.addOption("Controller", kController);
		SmartDashboard.putData("Controller type", driveChooser);
		driveMode = "Joystick";
		
	}

	@Override
	public void robotPeriodic() {
		driveMode = driveChooser.getSelected();
		switch(driveMode){
			default:
			xAxisDrive = ControllerButtons.xAxisDrive;
			yAxisDrive = ControllerButtons.yAxisDrive;
			gTurnAxis = ControllerButtons.gTurnAxis;
			clawToggle = ControllerButtons.clawToggle;
			visionCone = ControllerButtons.visionCone;
			visionCube = ControllerButtons.visionCube;
			shifter = ControllerButtons.shifter;	
			driveStyle = ControllerButtons.driveStyle;
			break;

			case kJoystick:
			xAxisDrive = JoystickButtons.xAxisDrive;
			yAxisDrive = JoystickButtons.yAxisDrive;
			gTurnAxis = JoystickButtons.gTurnAxis;
			clawToggle = JoystickButtons.clawToggle;
			visionCone = JoystickButtons.visionCone;
			visionCube = JoystickButtons.visionCube;
			shifter = JoystickButtons.shifter;	
			driveStyle = JoystickButtons.driveStyle;			
			break;
		}


		}

	@Override
	public void autonomousInit(){
		autonomousTimer = 0;
		m_swerve.resetOdometry();
		currentPath = getPath(autoExample);

	}

	@Override
	public void autonomousPeriodic() {

		//driveWithJoystick(false);
		m_swerve.updateOdometry();

		followTrajectory(autonomousTimer, currentPath);

		autonomousTimer += 0.02;
	}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
		m_swerve.updateOdometry();
		

		switch (var) {
			case 0:
				drive(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis), true);
				System.out.print("reg drive");
				if(m_controller.getRawButton(driveStyle))
					var++;
			break;
			
			case 1:
				drive(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis), true);
				if(! m_controller.getRawButton(driveStyle))
					var++;
					break;

			case 2:
				driveAngle(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis) * 180, true);
				System.out.print("not reg drive");
				if(m_controller.getRawButton(driveStyle))
					var++;
				break;
			
			case 3:
				driveAngle(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis) * 180, true);
				if( !m_controller.getRawButton(driveStyle))
				var = 0;
				break;
		}
	

		speedShift(m_controller.getRawButtonReleased(shifter));
		
		
	}

		
	public void speedShift(boolean getRawButtonReleased){
		/*
		switch(speedVar){
			case 0:
				if(m_controller.getRawButtonReleased(changeSpeedButton)){
					speedMult = 1;
					speedVar++;
				}
			break;
			case 1: 
				if(m_controller.getRawButtonReleased(changeSpeedButton)){
					speedMult = 0.25;
					speedVar = 0;
				}
			break;
		}
		*/

		switch(speedVar){
			case 0:
				speedMult = 1;
				if(m_controller.getRawButton(changeSpeedButton)){
					speedVar++;
				}
				break;

			case 1:
			if(!m_controller.getRawButton(changeSpeedButton)){
				speedVar++;
			}
			break;

			case 2:
				speedMult = 0.25;
				if(m_controller.getRawButton(changeSpeedButton)){
					speedVar++;
				}
				break;

			case 3:
				if(!m_controller.getRawButton(changeSpeedButton)){
					speedVar = 0;
				}
				break;
		}
	}

	public void driveAngle(double xSpeed, double ySpeed, double rot, boolean fieldRelative){

		double[] arr = deadZone(xSpeed, ySpeed, 0.2);
		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(arr[0], 0.2));	
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(arr[1], 0.2));
		
		m_swerve.driveAngle(xSpeed * speedMult*3, ySpeed * speedMult*3, rot*6.28, fieldRelative);
	}
	
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.1));	
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.1));
		rot = m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.2));
		m_swerve.drive(-ySpeed * speedMult*3, -xSpeed * speedMult*3, rot*6.28, fieldRelative);
	}

	public double joyAngle(double x, double y){
		
		if(0.7 < Math.abs(x) || 0.7 < Math.abs(y) ){
			angleToHold = Math.toDegrees(Math.atan(y * (y/x)));
			if(y < 0)
				angleToHold = angleToHold + 180;	
	
		}
		return angleToHold;
	
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

	public static double[] deadZone(double xAxis, double yAxis, double deadZoneRange){
		double deadZoneArray[];
		deadZoneArray = new double[2];
		if(xAxis < deadZoneRange && yAxis < deadZoneRange){
			xAxis = 0;
			yAxis = 0;
		}
		deadZoneArray[0] = xAxis;
		deadZoneArray[1] = yAxis;
		return deadZoneArray;
	}
}

