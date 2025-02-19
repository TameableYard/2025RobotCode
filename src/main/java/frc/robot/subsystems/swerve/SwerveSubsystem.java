package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public SwerveSubsystem(File directory) {
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

        swerveDrive.useExternalFeedbackSensor();
    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        /**
         * Change this so the initial pose is fetched from the limelight and/or pathplanner path on startup
         */
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveConstants.MAX_SPEED, new Pose2d(new Translation2d(Meter.of(2), Meter.of(2)), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }

    
}
