// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// A robot arm subsystem that moves with a motion profile.
public class PivotSubsystem extends SubsystemBase {

  private final SparkMax pivotMotor = new SparkMax(PivotConstants.kPivotMotorPort, MotorType.kBrushless);
  
  private final SparkClosedLoopController pivotClosedLoopController = pivotMotor.getClosedLoopController();

  private final RelativeEncoder motorEncoder = pivotMotor.getEncoder();

  private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(4, 1, 0.935);//0.402 for 0 to be right as in unit circle
  
  
  /*pivotConfig.smartCurrentLimit(Constants.MotorLimit.Neo.stall,
  Constants.MotorLimit.Neo.free,
  Constants.MotorLimit.Neo.stallRPM);*/
  
  /*{
    for(var motor : List.of(pivotMotor) ) {
      pivotMotor.setSmartCurrentLimit(
          Constants.MotorLimit.Neo.stall,
          Constants.MotorLimit.Neo.free,
          Constants.MotorLimit.Neo.stallRPM);
    }
  }*/
  
  private final ProfiledPIDController pivotProfiledPIDController;

  /* 
  
  private ProfiledPIDController armProfiledPIDController = new ProfiledPIDController(
    PivotConstants.kP,
    0,
    0,
    new TrapezoidProfile.Constraints(
        PivotConstants.kMaxVelocityRadPerSecond,
        PivotConstants.kMaxAccelerationRadPerSecSquared),
    0.2);//, Arm.kStartingPos
        //add 0 
*/
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(
          PivotConstants.kSVolts, PivotConstants.kGVolts,
          PivotConstants.kVVoltSecondPerRad, PivotConstants.kAVoltSecondSquaredPerRad);

  //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Vision");


  // Create a new ArmSubsystem. 
  public PivotSubsystem() {

    /*Shuffleboard.getTab("Pivot")
        .add("Motor Encoder Pos", motorEncoder);
        Shuffleboard.getTab("Pivot")
        .add("Throughbore Encoder Pos", throughboreEncoder);*/

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig
      .smartCurrentLimit(Constants.MotorLimit.Neo.stall, Constants.MotorLimit.Neo.free,
    Constants.MotorLimit.Neo.stallRPM)
      .idleMode(IdleMode.kBrake)
      .inverted(PivotConstants.pivotMotorInverted)
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      
      .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
      .outputRange(-1, 1)
      .maxMotion
      
      .maxVelocity(PivotConstants.kMaxVelocityRadPerSecond)
      .maxAcceleration(PivotConstants.kMaxAccelerationRadPerSecSquared)
      .allowedClosedLoopError(PivotConstants.kPivotAllowedClosedLoopError)
      ;

    //TODO: fix this, it causes errors currently but is neede: pivotConfig.encoder.positionConversionFactor(1/27);
//TODO: figure out if the motor is inverted or not
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    synchronizeEncoders();

    pivotProfiledPIDController = new ProfiledPIDController(
            PivotConstants.kP,
            PivotConstants.kI,
            PivotConstants.kD,
            new TrapezoidProfile.Constraints(
                PivotConstants.kMaxVelocityRadPerSecond,
                PivotConstants.kMaxAccelerationRadPerSecSquared));//, Arm.kStartingPos;//add 0 

    pivotProfiledPIDController.setTolerance(0.01);
    
    
    //throughboreEncoder.setPositionOffset(0.386);
    //m_encoder.setDistancePerRotation(PivotConstants.kEncoderDistancePerRotation);
    //throughboreEncoder.get();
    // Start arm at rest in neutral position
    //setGoal(Arm.kArmOffsetRads);
    //setGoal(PivotConstants.kStartingPos);

    //enable();
  }
/* 
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    pivotMotor.setVoltage(-output + feedforward);// + feedforward);
    //System.out.println("Absolute position encoder measurement: " + getMeasurement());
    //System.out.println("Voltage: " + output);
    
  }

  @Override
  public double getMeasurement() {
    return Math.PI*2*(m_encoder.getAbsolutePosition() - m_encoder.getPositionOffset()); //+ Arm.kArmOffsetRads;
    
  }*/

  public double showPositionError() {
    return pivotProfiledPIDController.getPositionError();
  }

  public void synchronizeEncoders() {
    motorEncoder.setPosition(Rotations.of(getThroughborePos()).in(Rotations));
  }

  public void reachSetpoint(double setPointDegree) {
    double  goalPosition = convertAngleToSensorUnits(Rotations.of(setPointDegree)).in(Rotations);
    boolean rioPID       = true;
    if (rioPID)
    {
      double pidOutput     = pivotProfiledPIDController.calculate(getThroughborePos(), goalPosition);
      State  setpointState = pivotProfiledPIDController.getSetpoint();

      SmartDashboard.putNumber("pidOutput", pidOutput);
      pivotMotor.setVoltage(pidOutput +
                         armFeedforward.calculate(Rotations.of(setpointState.position).in(Radians),
                                                 setpointState.velocity)
                        );
    } else
    {
      pivotClosedLoopController.setReference(goalPosition,
                                ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
  } 

  public static Angle convertAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations));// * PivotConstants.kPivotReduction);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations));// / PivotConstants.kPivotReduction);

    }

    public double getThroughborePos() {
      return Rotations.of(throughboreEncoder.get()).in(Rotations);
    }

    public double getThroughborePosRadians() {
      return Rotations.of(throughboreEncoder.get() - 0.473).in(Radians);
    }

    public double getMotorPos() {
      return motorEncoder.getPosition();
    }

}