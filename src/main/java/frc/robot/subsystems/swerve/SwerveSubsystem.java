package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    //private Vision vision;

    private boolean blueAlliance;

    private RobotConfig config;

    private Pose2d startingPose;

    Limelight limelight;

    LimelightPoseEstimator limelightPoseEstimator;

    Optional<PoseEstimate> poseEstimates;

    private final boolean useVision = false; //TODO: change once limelight is reattached
    
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    StructPublisher<Pose2d> visionpublisher = NetworkTableInstance.getDefault().getStructTopic("vision pose", Pose2d.struct).publish();

    public SwerveSubsystem(File directory) {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) { //TODO: add exception handling for no alliance found
            blueAlliance = false;
        } else {
            blueAlliance = true;
        }

        startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED, startingPose);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false); //only use if controlling bot via angle

        if (SwerveDriveTelemetry.isSimulation) {
            swerveDrive.setCosineCompensator(false);
        } else {
            swerveDrive.setCosineCompensator(true);
        }

        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

        //swerveDrive.setAngularV

        //TODO: set angular velocity compensation for skew correction after rotating

        swerveDrive.pushOffsetsToEncoders();

        if (useVision) {
            limelightSetup();
            //stop odometry thread when using vision so updates can be synchronized better
            swerveDrive.stopOdometryThread();


            
        }
        setupPathPlanner();

        
        /*try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

    setupPathPlanner();*/
    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        /**
         * Change this so the initial pose is fetched from the limelight and/or pathplanner path on startup
         */
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveConstants.MAX_SPEED, startingPose);//new Pose2d(new Translation2d(Meter.of(16.38), Meter.of(6.03)), Rotation2d.fromDegrees(0)));
    }

    private int outOfAreaReading = 0;
    private boolean initialReading = false;

    @Override
    public void periodic() {
        if (useVision) {
            //swerveDrive.updateOdometry();
            //vision.updatePoseEstimation(swerveDrive);
            updatePoseEstimation();
        }

        //swerveDrive.updateOdometry();

        //swerveDrive.getPose
        publisher.set(swerveDrive.getPose());
        //arrayPublisher.set(new)
        
    }



public void setupPathPlanner() {
    //load the RobotConfig from the GUI settings
    //TODO: store in Constants file
    RobotConfig config;
    try {
        config = RobotConfig.fromGUISettings();

        final boolean enableFeedforward = true;
        //configure autobuilder last
        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            (speedsRobotRelative, moduleFeedForwards) -> {
                if (enableFeedforward) {
                    swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                    );
                } else {
                    swerveDrive.setChassisSpeeds(speedsRobotRelative);
                }
            },
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
            } catch (Exception e) {
                e.printStackTrace();
            }

            PathfindingCommand.warmupCommand().schedule();
    
} 

  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
    //swerveDrive.drive();
    
  }

//  public Command runBackCommand()

    


private static double adjustSensitivity(double value) {
    double sign = Math.signum(value);
    double absvalue = Math.abs(value);
    value = sign * Math.pow(absvalue, Constants.DriverConstants.kSensitivity);
    return value;
}

public ChassisSpeeds getTargetSpeeds(double xInput, 
                                     double yInput,
                                     Rotation2d angle) {
                                        xInput = adjustSensitivity(xInput);
                                        yInput = adjustSensitivity(yInput);
                                        
                                        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), SwerveConstants.MAX_SPEED);
                                     }

public Pose2d getPose() {
    return swerveDrive.getPose();
}

public Rotation2d getHeading() {
    return getPose().getRotation();
}

public SwerveDrive getSwerveDrive() {
    return swerveDrive;
}

/**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
}

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
}

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
}

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
}

public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }
  
public void zeroGyro() {
    swerveDrive.zeroGyro();
}

public void resetOdometry(Pose2d initialHolonomicPose){
  swerveDrive.resetOdometry(initialHolonomicPose);
}

public void limelightSetup() {
    swerveDrive.stopOdometryThread();
    limelight = new Limelight("limelight");
    limelight.getSettings()
             .withPipelineIndex(0)
             .withCameraOffset(VisionConstants.LIMELIGHT_POSE)
             .save();
    
    limelightPoseEstimator = limelight.getPoseEstimator(true);
            
}

public void updatePoseEstimation() {
    limelight.getSettings()
    .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
        new AngularVelocity3d(DegreesPerSecond.of(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityXDevice().getValueAsDouble()),
                              DegreesPerSecond.of(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityYDevice().getValueAsDouble()),
                              DegreesPerSecond.of(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityZDevice().getValueAsDouble()))))
             
             .save();

    poseEstimates = limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results = limelight.getLatestResults();

    poseEstimates.ifPresent((PoseEstimate poseEstimate) -> {
        if (poseEstimates.get().tagCount > 0) {
            // Add it to the pose estimator as long as robot is rotating at less than 720 degrees per second
            if (Math.abs(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityYDevice().getValueAsDouble()) < Math.toRadians(720)){
                
                //SmartDashboard.putData("visionpose", (Sendable) poseEstimate.pose.toPose2d());
                swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
                swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
            }
        }
    });
/*
    if (results.isPresent()) {
        LimelightResults result = results.get();
        PoseEstimate poseEstimate = poseEstimates.get();
        SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
        SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
        SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
        SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
        SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
        SmartDashboard.putNumber("Odom Pose/x", swerveDrive.getPose().getX());
        SmartDashboard.putNumber("Odom Pose/y", swerveDrive.getPose().getY());
        SmartDashboard.putNumber("Odom Pose/degrees", swerveDrive.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
        SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
        SmartDashboard.putNumber("Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
        if (result.valid) {
            Pose2d blueSidePose = result.getBotPose2d(Alliance.Blue);
            

            //SmartDashboard.putData("visionpose", Sendable<blueSidePose>);
            //double distanceToPose = blueSidePose.getTranslation().getDistance(swerveDrive.getPose().getTranslation());
            if (poseEstimate.tagCount > 0) {
                visionpublisher.set(blueSidePose);
                swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
                swerveDrive.addVisionMeasurement(blueSidePose, Timer.getTimestamp());
            }
            if (distanceToPose < 0.5 || (outOfAreaReading > 10) || (outOfAreaReading > 10 && !initialReading)) {
                if (!initialReading) {
                    initialReading = true;
                }
                outOfAreaReading = 0;
                swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
                swerveDrive.addVisionMeasurement(blueSidePose, Timer.getTimestamp());
            } else {
                outOfAreaReading += 1;
            }
        
        }
    }*/
    swerveDrive.updateOdometry();
    
}
/*
public void updateRobotOrientation(SwerveDrive swerveDrive) {
        limelight.getSettings().withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
        new AngularVelocity3d(DegreesPerSecond.of(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityXDevice().getValueAsDouble()),
                              DegreesPerSecond.of(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityYDevice().getValueAsDouble()),
                              DegreesPerSecond.of(((Pigeon2) swerveDrive.getGyro().getIMU()).getAngularVelocityZDevice().getValueAsDouble()))));
    }*/

}


