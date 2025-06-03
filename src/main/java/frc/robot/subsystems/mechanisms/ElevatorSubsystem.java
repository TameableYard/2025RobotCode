// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Heights;

public class ElevatorSubsystem extends SubsystemBase
{

  public final Trigger atMin = new Trigger(() -> getLinearPosition().isNear(ElevatorConstants.kMinElevatorHeight,
                                                                            Inches.of(24)));
  public final Trigger atMax = new Trigger(() -> getLinearPosition().isNear(ElevatorConstants.kMaxElevatorHeight,
                                                                            Inches.of(12)));

  public double desiredHeight = ElevatorConstants.kBottom;

  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  private final SparkMax m_BackMotor    = new SparkMax(ElevatorConstants.kBackMotorPort, MotorType.kBrushless);
  private final SparkMax m_FrontMotor = new SparkMax(ElevatorConstants.kFrontMotorPort, MotorType.kBrushless);
  
  private final RelativeEncoder m_BackEncoder  = m_BackMotor.getEncoder();

  private final RelativeEncoder m_FrontEncoder = m_FrontMotor.getEncoder();

  private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp,
                                                                               ElevatorConstants.kElevatorKi,
                                                                               ElevatorConstants.kElevatorKd,
                                                                               new Constraints(ElevatorConstants.kMaxVelocity,
                                                                                               ElevatorConstants.kMaxAcceleration));

  // Simulation classes help us simulate what's going on, including gravity.



  // SysId Routine and seutp
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  private final MutAngle          m_rotations      = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);
  // SysID Routine
  private final SysIdRoutine      m_sysIdRoutine   =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.per(Second).of(0.75),
                                  Volts.of(5.5),
                                  Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              m_BackMotor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("elevator-improved")
                   .voltage(
                       m_appliedVoltage.mut_replace(
                           m_BackMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(getHeightMeters(),
                                                          Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                   .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                                                          MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
              },
              this));

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig frontFollowerConfig = new SparkMaxConfig();
    config
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
        
    m_BackMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    frontFollowerConfig.apply(config).follow(m_BackMotor);

    m_FrontMotor.configure(frontFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim

    /*if (RobotBase.isSimulation())
    {
      m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
      SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchLow);
    }*/
    //seedElevatorMotorPosition();
  }

  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  /*public void seedElevatorMotorPosition()
  {
    
      m_encoder.setPosition(convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCan.getMeasurement().distance_mm + ElevatorConstants.kLaserCANOffset.in(Millimeters)))
                                    .in(Rotations));
  }*/

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    double voltsOut = MathUtil.clamp(
        m_controller.calculate(getHeightMeters(), goal) +
        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              m_controller.getSetpoint().velocity), -10, 10);
    m_BackMotor.setVoltage(voltsOut);

    SmartDashboard.putNumber("elevatorPIDVoltage: ", m_controller.calculate(getHeightMeters(), goal));
    SmartDashboard.putNumber("elevatorFeedforwardVoltage: ", (m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
    m_controller.getSetpoint().velocity)));
    SmartDashboard.putNumber("elevatorTotalVoltage: ", voltsOut);
  }

  /**
   * Fake control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachFakeGoal(double fakeGoal)
  {
    double voltsOut = MathUtil.clamp(
        m_controller.calculate(getHeightMeters(), fakeGoal) +
        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              m_controller.getSetpoint().velocity), -10, 10);
    SmartDashboard.putNumber("elevatorPIDVoltage: ", m_controller.calculate(getHeightMeters(), fakeGoal));
    SmartDashboard.putNumber("elevatorFeedforwardVoltage: ", (m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
    m_controller.getSetpoint().velocity)));
    SmartDashboard.putNumber("elevatorTotalVoltage: ", voltsOut);
  }

  public double frontMCAppliedOutput() {
    return m_FrontMotor.getAppliedOutput();
  }

  public double backMCAppliedOutput() {
    return m_BackMotor.getAppliedOutput();
  }

  public void stopMotors() {
    m_BackMotor.setVoltage(0);
  }

  /**
   * Runs the SysId routine to tune the Elevator
   *
   * @return SysId Routine command
   */
  public Command runSysIdRoutine()
  {
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }


  /**
   * Get Elevator Velocity
   *
   * @return Elevator Velocity
   */
  public LinearVelocity getLinearVelocity()
  {
    return convertRotationsToDistance(Rotations.of(m_BackEncoder.getVelocity())).per(Minute);
  }

  /**
   * Get the height of the Elevator
   *
   * @return Height of the elevator
   */
  public Distance getLinearPosition()
  {
    return convertRotationsToDistance(Rotations.of(m_BackEncoder.getPosition()));
  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeightMeters()
  {
    return (m_BackEncoder.getPosition() / ElevatorConstants.kElevatorGearing) *
           (2 * Math.PI * ElevatorConstants.kElevatorLowerDrumRadius);
  }

  /**
   * Get the height in meters using the secondary encoder.
   *
   * @return Height in meters
   */
  public double getHeightMetersFrontEncoder()
  {
    return (m_FrontEncoder.getPosition() / ElevatorConstants.kElevatorGearing) *
           (2 * Math.PI * ElevatorConstants.kElevatorLowerDrumRadius); //TODO: change
  }

  /**
   * The velocity of the elevator in meters per second.
   *
   * @return velocity in meters per second
   */
  public double getVelocityMetersPerSecond()
  {
    return ((m_BackEncoder.getVelocity() / 60)/ ElevatorConstants.kElevatorGearing) *
           (2 * Math.PI * ElevatorConstants.kElevatorLowerDrumRadius); //TODO: change
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeightMeters(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations) {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       (ElevatorConstants.kElevatorLowerDrumRadius * 2 * Math.PI)); //TODO: change
    }

/**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance)
    {
      return Rotations.of(distance.in(Meters) /
                          (ElevatorConstants.kElevatorLowerDrumRadius * 2 * Math.PI) * //TODO: change
                          ElevatorConstants.kElevatorGearing);
    }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_BackMotor.set(0.0);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
  }

  public void changeDesiredHeight(Heights height) {
    switch (height) {
      case L1:
        desiredHeight = ElevatorConstants.kL1Height;
        break;
      case L2:
        desiredHeight = ElevatorConstants.kL2Height;
        break;
      case L3:
        desiredHeight = ElevatorConstants.kL3Height;
        break;
      case L4:
        desiredHeight = ElevatorConstants.kL4Height;
        break;
      case REST:
        desiredHeight = ElevatorConstants.kBottom;
        break;
    }
  }

  @Override
  public void periodic() {
   //reachGoal(desiredHeight);

  }
}