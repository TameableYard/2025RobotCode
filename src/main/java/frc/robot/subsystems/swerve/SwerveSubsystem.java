package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    private Vision vision;

    private boolean blueAlliance;

    private Pose2d startingPose;

    private final boolean useVision = false; //change once limelight is on production bot
    
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    public SwerveSubsystem(File directory) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { //TODO: add exception handling for no alliance found
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
            swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED);
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
            setupVision();
            //stop odometry thread when using vision so updates can be synchronized better
            swerveDrive.stopOdometryThread();
        }
        //setupPathPlanner();
    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        /**
         * Change this so the initial pose is fetched from the limelight and/or pathplanner path on startup
         */
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveConstants.MAX_SPEED, startingPose);//new Pose2d(new Translation2d(Meter.of(16.38), Meter.of(6.03)), Rotation2d.fromDegrees(0)));
    }

    public void setupVision() {
        //add the vision shtuff here later
        vision = new Vision(swerveDrive);
    }

    //TODO: add publishing of pose for advantagescope visualization, per https://docs.advantagescope.org/tab-reference/3d-field

    @Override
    public void periodic() {
        if (useVision) {
            swerveDrive.updateOdometry();
            vision.updatePoseEstimation(swerveDrive);
        }

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

}

