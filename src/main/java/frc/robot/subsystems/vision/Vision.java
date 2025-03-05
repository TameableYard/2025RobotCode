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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.LEDMode;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision {

    Limelight limelight;
    LimelightSettings limelightSettings;
    LimelightPoseEstimator poseEstimator;

    Orientation3d robotOrientation3d;
    
    public Pigeon2 getPigeon2(SwerveDrive swerveDrive) {
        return (Pigeon2) swerveDrive.getGyro().getIMU();
   }


    public Vision(SwerveDrive swerveDrive) {
        limelight = new Limelight("limelight");
        limelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl)
                                .withCameraOffset(VisionConstants.LIMELIGHT_POSE)/* */
                                .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
                                                                       new AngularVelocity3d(DegreesPerSecond.of(getPigeon2(swerveDrive).getAngularVelocityXDevice().getValueAsDouble()),
                                                                                             DegreesPerSecond.of(getPigeon2(swerveDrive).getAngularVelocityYDevice().getValueAsDouble()),
                                                                                             DegreesPerSecond.of(getPigeon2(swerveDrive).getAngularVelocityZDevice().getValueAsDouble()))))
                                .withImuMode(LimelightSettings.ImuMode.InternalImu)
                                .save();

        

        
        
    }

    public void updatePoseEstimation(SwerveDrive swerveDrive) {

        //update robot orientation

        updateRobotOrientation(swerveDrive);

        // Get MegaTag2 pose
        Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(true).getPoseEstimate();
        // If the pose is present
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            if (visionEstimate.get().tagCount > 0) {
                // Add it to the pose estimator as long as robot is rotating at less than 720 degrees per second
                if (Math.abs(getPigeon2(swerveDrive).getAngularVelocityYDevice().getValueAsDouble()) < Math.toRadians(720)){
                    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); no stddevs as for some reason yall does not (seemingly) support setting stddevs for vision measurements yet
                    swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
                }
            }
        });
    }

    public void getPoseEstimationMegatag1(SwerveDrive swerveDrive) {
        // Get MegaTag1 pose
        Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(false).getPoseEstimate();
        // If the pose is present
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            if (visionEstimate.get().tagCount > 0) {
                // Add it to the pose estimator as long as robot is rotating at less than 720 degrees per second
                if (Math.abs(getPigeon2(swerveDrive).getAngularVelocityYDevice().getValueAsDouble()) < Math.toRadians(720)){
                    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); no stddevs as for some reason yall does not (seemingly) support setting stddevs for vision measurements yet
                    swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
                }
            }
        });
    }

    public void updateRobotOrientation(SwerveDrive swerveDrive) {
        limelight.getSettings().withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
        new AngularVelocity3d(DegreesPerSecond.of(getPigeon2(swerveDrive).getAngularVelocityXDevice().getValueAsDouble()),
                              DegreesPerSecond.of(getPigeon2(swerveDrive).getAngularVelocityYDevice().getValueAsDouble()),
                              DegreesPerSecond.of(getPigeon2(swerveDrive).getAngularVelocityZDevice().getValueAsDouble()))));
    }
    

    

}
