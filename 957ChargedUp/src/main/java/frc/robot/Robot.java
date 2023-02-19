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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerButtons;
import frc.robot.Constants.JoystickButtons;

public class Robot extends TimedRobot {
	private final Drivetrain m_swerve = new Drivetrain();
	
	private final Limelight m_limelight = new Limelight();
	
	private final Wrist m_wrist = new Wrist();
	private final FourBar m_fourBar = new FourBar();
	private final Claw m_claw = new Claw();

	private final VisionSubsystem m_vision = new VisionSubsystem();

	String autoExample = "pathplanner/generatedJSON/basic.wpilib.json";
	Trajectory trajectory = new Trajectory();
	double autonomousTimer = 0;

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
	final int holdAngle = 0;
	boolean holdAngleSwitch = false;

	private final Joystick m_controller = new Joystick(0);

	private static final String kJoystick = "Joystick";
	private static final String kController = "Controller";
	private String driveMode;
	private final SendableChooser<String> driveChooser = new SendableChooser<>();

	//CONTROLLER DRIVE
	int xAxisDrive = 0;
	int yAxisDrive = 0;
	int gTurnAxis = 0;
	int clawToggle = 0;
	int shifter = 0;
	int driveStyle = 0;
	int clawIntake = 0;
	int clawOuttake;

	private final ButtonPanel m_buttonPanel = new ButtonPanel();

	int driveVar = 0;
	int trackingVar = 0;
	int clawVar = 0;
	

	Trajectory currentPath;

	//function for HDC
	public void followTrajectory(double time, Trajectory trajectory){
		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds adjustedSpeeds = controller.calculate(m_swerve.getPose(), goal, Rotation2d.fromDegrees(0));
		if(time > trajectory.getTotalTimeSeconds()){
			m_swerve.drive(0, 0, 0, false);
		}
		else{
			m_swerve.autoDrive(adjustedSpeeds);
		}
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
			xAxisDrive = ControllerButtons.xAxisDrive; //0
			yAxisDrive = ControllerButtons.yAxisDrive; //1
			gTurnAxis = ControllerButtons.gTurnAxis;   //4
			clawToggle = ControllerButtons.clawToggle; //1
			shifter = ControllerButtons.shifter;	   //2
			driveStyle = ControllerButtons.driveStyle; //8
			clawIntake = ControllerButtons.clawIntake; //5
			clawOuttake = ControllerButtons.clawOuttake; //6
			break;

			/* 
			case kJoystick:
			xAxisDrive = JoystickButtons.xAxisDrive; //0 
			yAxisDrive = JoystickButtons.yAxisDrive; //1
			gTurnAxis = JoystickButtons.gTurnAxis;   //2
			clawToggle = JoystickButtons.clawToggle; //1
			shifter = JoystickButtons.shifter;		 //12
			driveStyle = JoystickButtons.driveStyle; //11	
			break;
			*/
		}

		m_vision.maintainTX(m_limelight);

	//	SmartDashboard.putBoolean("switchEnabled", m_claw.m_limitSwitch.get());
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
		speedShift(m_controller.getRawButtonReleased(shifter));
		driveMode(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis));


		if(m_controller.getRawButton(clawIntake)){
			m_claw.clawIntake(.5);
		}else if(m_controller.getRawButton(clawOuttake)){
			m_claw.clawOuttake(.5);
		}else{
			m_claw.clawStop();
		}

		switch(clawVar){
			case 0:
			m_claw.coneMode();
			if(m_controller.getRawButton(clawToggle))
				clawVar = 1;
			break;

			case 1:
			m_claw.coneMode();
			if(!m_controller.getRawButton(clawToggle))
				clawVar = 2;
			break;

			case 2:
			m_claw.cubeMode();
			if(m_controller.getRawButton(clawToggle))
				clawVar = 3;
			break;

			case 3:
			m_claw.cubeMode();
			if(!m_controller.getRawButton(clawToggle))
				clawVar = 0;
			break;
		}
		/*
		switch(trackingVar){
			case 0:
				m_vision.resetCases();
				driveMode(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis));
				if(m_buttonPanel.visionConePressed())//cone
					trackingVar = 1;
				if(m_buttonPanel.visionCubePressed())//cube
					trackingVar = 3;
			break;

			case 1://cone
				driveMode(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis));
				if(!m_buttonPanel.visionConePressed())//cone
					trackingVar = 2;
			break;
			
			case 2://cone
				m_vision.TapeTracking(m_swerve, m_limelight);
				if(m_buttonPanel.visionConePressed() || m_buttonPanel.visionCubePressed())
					trackingVar = 5;
			break;

			case 3://cube
				driveMode(m_controller.getRawAxis(xAxisDrive), m_controller.getRawAxis(yAxisDrive), m_controller.getRawAxis(gTurnAxis));
				if(!m_buttonPanel.visionCubePressed())
					trackingVar = 4;
			break;

			case 4://cube
				m_vision.AprilTagTracking(m_swerve, m_limelight);
				if(m_buttonPanel.visionConePressed() || m_buttonPanel.visionCubePressed())
					trackingVar = 5;
			break;

			case 5:
				if(!m_buttonPanel.visionConePressed() && !m_buttonPanel.visionCubePressed())
					trackingVar = 0;
			break;

		}
	
		speedShift(m_controller.getRawButtonReleased(shifter));
		//MECHANISM CODE BELOW HERE
		
		
	
		
*/
		if(m_buttonPanel.wristRetractPressed())
			m_wrist.set(WristPositions.retract);
		if(m_buttonPanel.wristScoreUpPressed())
			m_wrist.set(WristPositions.scoreUp);
		if(m_buttonPanel.wristScoreOutPressed())
			m_wrist.set(WristPositions.scoreOut);
		if(m_buttonPanel.wristGroundPressed())
			m_wrist.set(WristPositions.ground);
		m_wrist.run();

		if(m_buttonPanel.armHighPressed())
			m_fourBar.setLevel(MoveFourBars.high);
		if(m_buttonPanel.armMidPressed())
			m_fourBar.setLevel(MoveFourBars.mid);
		if(m_buttonPanel.armSubPressed())
		m_fourBar.setLevel(MoveFourBars.substation);
		if(m_buttonPanel.armGroundPressed())
			m_fourBar.setLevel(MoveFourBars.ground);
		m_fourBar.run();

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
				if(m_controller.getRawButton(shifter)){
					speedVar++;
				}
				break;

			case 1:
			if(!m_controller.getRawButton(shifter)){
				speedVar++;
			}
			break;

			case 2:
				speedMult = 0.25;
				if(m_controller.getRawButton(shifter)){
					speedVar++;
				}
				break;

			case 3:
				if(!m_controller.getRawButton(shifter)){
					speedVar = 0;
				}
				break;
		}
	}

	public void driveAngle(double xSpeed, double ySpeed, double rot, boolean fieldRelative){

		double[] arr = deadZone(xSpeed, ySpeed, 0.2);
		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(arr[0], 0.2));	
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(arr[1], 0.2));
		
		m_swerve.driveAngle(-ySpeed * speedMult*3, -xSpeed * speedMult*3, -rot, fieldRelative);
	}
	
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
		xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.05));	
		ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.05));
		rot = m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.2));
		m_swerve.drive(-ySpeed * speedMult*3, -xSpeed * speedMult*3, rot*6.28, fieldRelative);
	}

	public double joyAngle(double x, double y){
		double a;
		double b;
		double c;
		double d;
		double e;

		if(0.7 < Math.abs(x) || 0.7 < Math.abs(y) ){
			angleToHold = Math.toDegrees(Math.atan2(y, x));	
		}
		if(angleToHold < 0){
			a = angleToHold;
			b = angleToHold + 360;
		}else{
			b = angleToHold;
			a = angleToHold - 360;
		}
		c = (m_swerve.m_navx.getAngle() - (m_swerve.m_navx.getAngle() % 360)) / 360;
		d = (360 * c) + a;
		e = (360 * c) + b;

		if(Math.abs(m_swerve.m_navx.getAngle() - d) <= 180){
			angleToHold = a;
		}else{
			angleToHold = b;
		}
		
		return angleToHold;
	
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

	public void driveMode(double x, double y, double rot){
		switch (driveVar) {
			case 0:
				drive(x, y, rot, true);
				if(m_controller.getRawButton(driveStyle))
					driveVar++;
			break;
			
			case 1:
				drive(x, y, rot, true);
				if(! m_controller.getRawButton(driveStyle))
					driveVar++;
			break;

			case 2:
				driveAngle(x, y, rot, true);
				if(m_controller.getRawButton(driveStyle))
					driveVar++;
				break;
			
			case 3:
				driveAngle(x, y, rot, true);
				if( !m_controller.getRawButton(driveStyle))
					driveVar = 0;
				break;
		}

	}
}

