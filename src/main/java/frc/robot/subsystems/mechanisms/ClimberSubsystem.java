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
public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax climberMotor = new SparkMax(40, MotorType.kBrushless);

  private final RelativeEncoder motorEncoder = climberMotor.getEncoder();

  private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(2, 1, 0.935);//0.402 for 0 to be right as in unit circle
  
  
private double offset = 0;



  //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Vision");


  // Create a new ArmSubsystem. 
  public ClimberSubsystem() {

    /*Shuffleboard.getTab("Pivot")
        .add("Motor Encoder Pos", motorEncoder);
        Shuffleboard.getTab("Pivot")
        .add("Throughbore Encoder Pos", throughboreEncoder);*/

    //TODO: fix this, it causes errors currently but is neede: pivotConfig.encoder.positionConversionFactor(1/27);
//TODO: figure out if the motor is inverted or not

    
    
    //throughboreEncoder.setPositionOffset(0.386);
    //m_encoder.setDistancePerRotation(PivotConstants.kEncoderDistancePerRotation);
    //throughboreEncoder.get();
    // Start arm at rest in neutral position
    //setGoal(Arm.kArmOffsetRads);
    //setGoal(PivotConstants.kStartingPos);

    //enable();
  }

  public void noClimber() {
    climberMotor.set(0);
  }

  public void climberIn() {
    climberMotor.set(-0.6);
  }

  public void climberOut() {
    climberMotor.set(0.6);
  }
  
    

    public double getThroughborePos() {
      return Rotations.of(throughboreEncoder.get() - offset).in(Rotations);
    }

    public double getThroughborePosRadians() {
      return Rotations.of(throughboreEncoder.get() - offset).in(Radians); // was 0.473
    }

    public void changeThroughbore() {
        offset = throughboreEncoder.get();
    }



}

