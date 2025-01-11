package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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

        swerveDrive.pushOffsetsToEncoders();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }

    
}
