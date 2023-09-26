// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static Robot instance;
  private RobotYK m_robot = new RobotYK();
  private Timer disabledTimer;

  private Spark leftmotor1 = new Spark(0);
  private Spark leftmotor2 = new Spark(1);
  private Spark rightmotor1 = new Spark(2);
  private Spark rightmotor2 = new Spark(3);

  private Joystick joy1 = new Joystick(0);

  
  private double startTime;

  private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(1);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);

  private WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private WPI_VictorSPX armSlave = new WPI_VictorSPX(3);

  private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);

  private Compressor compressor = new Compressor(null);
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(null, 0, 0);
  //PCM PORT 

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);


  //joystick or contr
  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  //unit-conv
  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double KArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;

  /** Robot constructor */
  public Robot() {
    instance = this;

  }


  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    startTime = Timer.getFPGATimestamp();

    // setit
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    // slave
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    //init-encode

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSensorPhase(true);

    //reset-encode

    leftMaster.setSelectedSensorPosition(0,10,10);
    rightMaster.setSelectedSensorPosition(0,0,10);
    armMotor.setSelectedSensorPosition(0,0,10);

    //boundary-limit-to-stop-motor
    armMotor.configReverseSoftLimitThreshold((int) (0 / KArmTick2Deg), 10);
    armMotor.configReverseSoftLimitThreshold((int) (175 / KArmTick2Deg), 10);

    armMotor.configReverseSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);

    // compressor

    compressor.start();

    drive.setDeadband(0.05);
    // buradan sonrası biraz farklı

    m_robotContainer = new RobotContainer();

    // Disable joystick warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    m_robot.configureButtonBindings();
    m_robot.configureAxisActions();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot
    // stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition() * KArmTick2Deg);
    SmartDashboard.putNumber("Left Drive Encoder Value", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder Value", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robot.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    m_robot.setMotorBrake(true);

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //res-encoders-to-zero
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    enableMotors(true);
    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

      m_robot.setMotorBrake(true);
    m_autonomousCommand = m_robot.getAutonomousCommand();

    // schedule the autonomous command 
    if (m_autonomousCommand != null); }
        m_autonomousCommand.schedule();
 
  }

  final double kP = 0.5;
  final double kI = 5;
  final double kD = 0.01;
  final double iLimit = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

    if(time - startTime < 3 ){

    leftmotor1.set(0.6);
    leftmotor2.set(0.6);
    rightmotor1.set(-0.6);
    rightmotor2.set(-0.6);
    } else {
      leftmotor1.set(0);
      leftmotor2.set(0);
      rightmotor1.set(0);
      rightmotor2.set(0);
    }
    double leftPosition = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;
    
    if(distance < 10) {
      drive.tankDrive(0.6, 0.6);
    }else{
      drive.tankDrive(0, 0);
    }

    if (joy1.getRawButton(1)){
      setpoint = 10;
    }else if (joy1.getRawButton(2)) {
      setpoint = 0;

    }

    //sensor-position
    double sensorPosition = encoder.get() * kDriveTick2Feet;

    //calcu
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
    errorSum += error * dt;
    }


    double errorRate = (error - lastError) / dt;
    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;




    //output to motors
    leftmotor1.set(outputSpeed);
    leftmotor2.set(outputSpeed);
    rightmotor1.set(outputSpeed);
    rightmotor2.set(outputSpeed);

    //update-lastvariables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  @Override
  public void teleopInit() {}

     /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
   */
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
   */
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
   */

  @Override
  public void teleopPeriodic() {
    double speed = -joy1.getRawAxis(1) * 0.6;
    double turn = joy1.getRawAxis(4) * 0.3;

    double left = speed + turn;
    double right = speed - turn;

    leftmotor1.set(left);
    leftmotor2.set(left);
    rightmotor1.set(-right);
    rightmotor2.set(-right);

    //drive
    double power = -driverJoystick.getRawAxis(1); // negative sign
    double turn = driverJoystick.getRawAxis(4);

    //deadband
    //  if (Math.abs(turn) < 0.05) {
    //   power = 0
    //  }
    //  if (Math.abs(turn) < 0.05) {
    //  turn = 0;
    // }


    drive.arcadeDrive(power * 0.6, turn * 0.3);

    //arm-control
    double armPower = -operatorJoystick.getRawAxis(1); //negative sign
    if (Math.abs(armPower) < 0.05) {
      armPower = 0;
    }
    armPower *= 0.5;
    armMotor.set(ControlMode.PercentOutput, armPower);

    //roller-cont
    int rollerPower;
    if (operatorJoystick.getRawButton(1)==true){
      rollerPower = 1;
    } else if (operatorJoystick.getRawButton(2)) {
      rollerPower = -1;
    }
    rollerMotor.set(ControlMode.PercentOutput, rollerPower);
    
    //hatch-intake
    if(operatorJoystick.getRawButton(3)) {
      hatchIntake.set(Value.kReverse);
    } else {
      hatchIntake.set(Value.kForward);
    }
    enableMotors(true);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    try {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    } 
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
  private void enableMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    }else{
      mode = NeutralMode.Coast;
    }
  

   leftMaster.setNeutralMode(mode);
   rightMaster.setNeutralMode(mode);
   leftSlave.setNeutralMode(mode);
   rightSlave.setNeutralMode(mode);
   armMotor.setNeutralMode(mode);
   armSlave.setNeutralMode(mode);
   rollerMotor.setNeutralMode(mode);

   enableMotors(false);
  
  }


  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
