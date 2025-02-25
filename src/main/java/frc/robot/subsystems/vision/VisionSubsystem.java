package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings;
import limelight.networktables.Orientation3d;
import limelight.networktables.LimelightSettings.LEDMode;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class VisionSubsystem extends SubsystemBase {

    Limelight limelight;
    LimelightSettings limelightSettings;
    LimelightPoseEstimator poseEstimator;
    private final SwerveSubsystem swerve;
    private final SwerveDriveOdometry3d swerveDriveOdometry3d = swerve.get
    private final Pigeon2 gyro = (Pigeon2) swerve.getSwerveDrive().getGyro().getIMU().;


    public VisionSubsystem() {
        limelight = new Limelight("limelight");
        limelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl)
                               .withCameraOffset(VisionConstants.LIMELIGHT_POSE)
                               .withRobotOrientation(new Orientation3d(swerve.getSwerveDrive().getGyro().getRotation3d(),
                                                                       new AngularVelocity3d(DegreesPerSecond.of(gyro.getRollVelocity()),
                                                                                             DegreesPerSecond.of(gyro.getRollVelocity()),
                                                                                             DegreesPerSecond.of(swerve.getSwerveDrive().getGyro().getYawAngularVelocity()))))
                               .save();
    }


    
    
    

}
