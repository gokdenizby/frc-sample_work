package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.frc.lib.logging.SpartanBooleanEntry;
import frc.robot.frc.lib.logging.SpartanDoubleEntry;
import frc.robot.frc.lib.util.CANSparkMaxUtil;
import frc.robot.frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final CANSparkMax leftElevatorMotor =
      new CANSparkMax(
          Constants.ElevatorConstants.leftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightElevatorMotor =
      new CANSparkMax(
          Constants.ElevatorConstants.rightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup elevatorGroup =
      new MotorControllerGroup(leftElevatorMotor, rightElevatorMotor);
  private final SparkMaxLimitSwitch limitSwitch =
      leftElevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private final Encoder elevatorEncoder =
      new Encoder(Constants.ElevatorConstants.encoderA, Constants.ElevatorConstants.encoderB, true);

  private boolean isClosedLoop;
  private TrapezoidProfile.State goal;
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ElevatorConstants.pGain,
          Constants.ElevatorConstants.iGain,
          Constants.ElevatorConstants.dGain,
          new TrapezoidProfile.Constraints(
              Constants.ElevatorConstants.maxVelocityMeterPerSecond,
              Constants.ElevatorConstants.maxAccelerationMeterPerSecondSquared));

  private final SlewRateLimiter limiter = new SlewRateLimiter(2.0);

  private SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Position");
  private SpartanDoubleEntry currGoalEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Current Goal");
  private SpartanBooleanEntry goalEntry =
      new SpartanBooleanEntry("/Diagnostics/Elevator/Goal Reached");
  private SpartanBooleanEntry reverseLimitEntry =
      new SpartanBooleanEntry("/Diagnostics/Elevator/Bottom Limit");
  private SpartanDoubleEntry leftOutputEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Left Output");
  private SpartanDoubleEntry rightOutputEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Right Output");

  public Elevator() {
    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftElevatorMotor, Usage.kAll);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightElevatorMotor, Usage.kMinimal);

    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    leftElevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.currentLimit);
    rightElevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.currentLimit);
    leftElevatorMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightElevatorMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightElevatorMotor.setInverted(true);
    elevatorGroup.setInverted(true);

    elevatorEncoder.setDistancePerPulse(Constants.ElevatorConstants.distancePerPulse);
    elevatorEncoder.setSamplesToAverage(Constants.ElevatorConstants.averageSampleSize);

    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    elevatorGroup.set(0.0);
    elevatorEncoder.reset();

    enable();
  }

  public void enable() {
    isClosedLoop = true;
    controller.reset(getPosition());
  }

  public void disable() {
    isClosedLoop = false;
    controller.setGoal(new State());
  }

  public Command setGoal(TrapezoidProfile.State state) {
    return runOnce(
            () -> {
              goal = state;
            })
        .until(controller::atGoal);
  }

  public CommandBase runElevator(DoubleSupplier elevator) {
    return run(
        () -> {
          if (!isClosedLoopEnabled()) {
            elevatorGroup.set(limiter.calculate(elevator.getAsDouble()));
          }
        });
  }

  public void controllerPeriodic() {
    if (isClosedLoopEnabled()) {
      if (goal != null) {
        controller.setGoal(
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    goal.position,
                    Constants.ElevatorConstants.minHeight,
                    Constants.ElevatorConstants.maxHeight),
                goal.velocity));
      } else {
        controller.setGoal(new State());
      }

      elevatorGroup.setVoltage(controller.calculate(getPosition(), controller.getGoal()));
    }
  }

  public boolean isClosedLoopEnabled() {
    return isClosedLoop;
  }

  public double getPosition() {
    return elevatorEncoder.getDistance();
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    positionEntry.set(getPosition());
    currGoalEntry.set(controller.getGoal().position);
    goalEntry.set(controller.atGoal());
    reverseLimitEntry.set(limitSwitch.isPressed());
    leftOutputEntry.set(leftElevatorMotor.getAppliedOutput());
    rightOutputEntry.set(rightElevatorMotor.getAppliedOutput());
  }
}
