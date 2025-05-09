// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriverConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kSensitivity = 3;
    public static final double DEADBAND = 0.3;
  }

  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 1;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double ROBOT_MASS = Units.lbsToKilograms(125);
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3.35)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    
    
  }

  public static class VisionConstants {
    public static final Pose3d LIMELIGHT_POSE = new Pose3d(0.0,
                                                            -0.2744,
                                                            0.2286,
                                                            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(32.005), Units.degreesToRadians(90)));
  }

  public static class MotorLimit {
    public static class Neo {
      public static final int stall = 60;
      public static final int free  = 40;
      public static final int stallRPM = 0; // 0 for linear limit
    }
    public static class Neo550 {
      public static final int stall = 40;
      public static final int free  = 20;
      public static final int stallRPM = 0; // 0 for linear limit
    }
  }

  public static class PivotConstants {
    public static final int kPivotMotorPort = 50;
    public static final double kMaxVelocityRadPerSecond = 0.25;
    public static final double kGVolts = 0.27; //calculated as 0.22 without coral
    public static final double kVVoltSecondPerRad = 0.53;
    public static final double kAVoltSecondSquaredPerRad = 0.01 / (Math.PI*2);
    public static final double kSVolts = 0.8; //try 0.8 if it doesn't work
    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMaxAccelerationRadPerSecSquared = 1;
    //public static final double kArmOffsetRads = Math.PI/2;
    public static final double kEncoderDistancePerRotation = 2*Math.PI;
    public static final double kPivotReduction = 27;
    public static final double kPivotAllowedClosedLoopError = (Rotations.of((Degrees.of(0.01)).in(Rotations))).in(Rotations);
    public static boolean pivotMotorInverted = false;

    public static final double kVerticalRot = 0.51; //0.48 formerly before chain tightening
    public static final double kStowRot = 0.81;//0.07;
    public static final double kOutRot = 0.55; //TODO: get value
    public static final double kInRot = 0.55; //TODO: get value
    public static final double kClimbRot = 0.44;//0.16;
    public static final double kScoreRot = 0.608;//0.33;
    public static final double kHumanPlayerStationRot = 0.36;//0.104;//0.36;//0.104;//0.55;

    
  }
  
  public static class ElevatorConstants {
    public static final int kBackMotorPort = 30;
    public static final int kFrontMotorPort = 31;

    public static final double kElevatorKp = 25;//10;//30;//26.722;;//16;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;//1.6047;

    public static final double kElevatorkS = 0.01964; // volts (V)
    public static final double kElevatorkV = 3.07;//894; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.09;//173; // volt per acceleration (V/(m/s²)) nh
    public static final double kElevatorkG = 0.91274; // volts (V)

    public static final double kElevatorGearing    = 6.88/3;//8.45/3; // was 2

    public static final double kElevatorSprocketTeeth = 22;
    public static final double kElevatorPitch = Units.inchesToMeters(0.25);
    public static final double kElevatorDrumRadius = (kElevatorSprocketTeeth * kElevatorPitch) / (2 * Math.PI);//Units.inchesToMeters(1.0);
    public static final double kCarriageMass       = 18.1; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kLaserCANOffset    = Inches.of(3);
    public static final Distance kStartingHeightSim = Meters.of(0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(1.6764); //1.1176


    public static double kElevatorRampRate = 0.1;
    public static int    kElevatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(0.75).per(Second).in(MetersPerSecond); //2.5 //0.5
    public static double kMaxAcceleration = Meters.of(1.5).per(Second).per(Second).in(MetersPerSecondPerSecond); //2

    public static double kL1Height = 0.45;
    public static double kL2Height = 0.66;
    public static double kL3Height = 1.04;
    public static double kL4Height = 1.64;//1.24; 1.64
    public static double kBottom = 0.0;
    public static double kHumanPlayerStation = 0.55;

    public static final double kSafetyHeight = 0.25;
  }

  public static class ShooterConstants {
    public static int kBigShooterMotorPort = 35;
    public static int kSmallShooterMotorPort = 36;

    public static double kFlywheelGearing = 4.0;

    public static double kBigFlywheelMomentOfInertia = 0.00012916529; //kg * m^2
    public static double kSmallFlywheelMomentOfInertia = 4.3228437e-5; //kg * m^2

    public static double kStateStdDevs = 3.0;
    public static double kMeasurementStdDevs = 0.01;
    public static double kQelms = 80.0;
    public static double kRelms = 12.0;

    public static double kMaxVoltage = 12.0;

    public static int kBigSpunUpRPM = 5500;
    public static int kSmallSpunUpRPM = 5500;

    public static int kBigIntakeSpeed = 375;
    public static int kSmallIntakeSpeed = 375;

    public static int kBigL1Speed = 250;
    public static int kSmallL1Speed = 375;

    public static int kBigL24Speed = 250;
    public static int kSmallL24Speed = 250;


  }

}
