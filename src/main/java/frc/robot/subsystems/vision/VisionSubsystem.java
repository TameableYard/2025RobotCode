package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;

/**
 * Vision subsystem for robot localization using Limelight camera
 * Provides AprilTag detection and pose estimation capabilities
 */
public class VisionSubsystem extends SubsystemBase {
    
    private final Limelight limelight;
    private final LimelightPoseEstimator poseEstimator;
    private final SwerveDrive swerveDrive;
    
    // Network Tables publishers for debugging
    private final StructPublisher<Pose2d> visionPosePublisher;
    
    // Vision update tracking
    private int consecutiveGoodUpdates = 0;
    private int consecutiveBadUpdates = 0;
    private static final int REQUIRED_GOOD_UPDATES = 3;
    private static final int MAX_BAD_UPDATES = 10;
    
    // Vision quality thresholds
    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_ROTATION_SPEED = Math.toRadians(720); // deg/s
    private static final double MAX_POSE_DIFFERENCE = 1.0; // meters
    
    // Standard deviations for vision measurements (lower = more trust)
    private static final double[] SINGLE_TAG_STD_DEVS = {1.0, 1.0, 2.0};
    private static final double[] MULTI_TAG_STD_DEVS = {0.5, 0.5, 1.0};
    private static final double[] HIGH_CONFIDENCE_STD_DEVS = {0.1, 0.1, 0.5};
    
    private boolean visionEnabled = true;
    
    /**
     * Creates a new VisionSubsystem
     * @param swerveDrive Reference to the swerve drive subsystem
     */
    public VisionSubsystem(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
        // Initialize Limelight
        limelight = new Limelight("limelight");
        
        // Configure Limelight settings
        limelight.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(VisionConstants.LIMELIGHT_POSE)
                .withPipelineIndex(0)
                .save();
        
        // Create pose estimator (MegaTag2 for better accuracy)
        poseEstimator = limelight.getPoseEstimator(true);
        
        // Set up Network Tables for visualization
        visionPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/EstimatedPose", Pose2d.struct)
            .publish();
        
        // Put settings on dashboard
        SmartDashboard.putBoolean("Vision/Enabled", visionEnabled);
        SmartDashboard.putNumber("Vision/MaxAmbiguity", MAX_AMBIGUITY);
    }
    
    @Override
    public void periodic() {
        // Check if vision should be enabled
        visionEnabled = SmartDashboard.getBoolean("Vision/Enabled", true);
        
        if (!visionEnabled) {
            return;
        }
        
        // Update robot orientation for Limelight
        updateRobotOrientation();
        
        // Get and process vision measurements
        processPoseEstimate();
        
        // Update dashboard
        updateTelemetry();
    }
    
    /**
     * Updates the robot orientation that Limelight uses for MegaTag
     */
    private void updateRobotOrientation() {
        Pigeon2 gyro = (Pigeon2) swerveDrive.getGyro().getIMU();
        
        limelight.getSettings()
                .withRobotOrientation(new Orientation3d(
                    swerveDrive.getGyro().getRotation3d(),
                    new AngularVelocity3d(
                        DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
                        DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble()),
                        DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble())
                    )
                ))
                .save();
    }
    
    /**
     * Processes pose estimates from Limelight and adds them to odometry
     */
    private void processPoseEstimate() {
        Optional<PoseEstimate> poseEstimateOpt = poseEstimator.getPoseEstimate();
        Optional<LimelightResults> resultsOpt = limelight.getLatestResults();
        
        if (!poseEstimateOpt.isPresent() || !resultsOpt.isPresent()) {
            consecutiveBadUpdates++;
            return;
        }
        
        PoseEstimate poseEstimate = poseEstimateOpt.get();
        LimelightResults results = resultsOpt.get();
        
        // Validate the pose estimate
        if (!isValidPoseEstimate(poseEstimate, results)) {
            consecutiveBadUpdates++;
            consecutiveGoodUpdates = 0;
            return;
        }
        
        // Get the pose adjusted for alliance
        Pose2d estimatedPose = getAllianceAdjustedPose(poseEstimate);
        
        // Check if robot is rotating too fast
        Pigeon2 gyro = (Pigeon2) swerveDrive.getGyro().getIMU();
        double rotationSpeed = Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble());
        
        if (rotationSpeed > MAX_ROTATION_SPEED) {
            SmartDashboard.putString("Vision/Status", "Rotating too fast");
            consecutiveBadUpdates++;
            return;
        }
        
        // Check pose difference from odometry
        double poseDifference = estimatedPose.getTranslation()
            .getDistance(swerveDrive.getPose().getTranslation());
        
        if (poseDifference > MAX_POSE_DIFFERENCE && consecutiveGoodUpdates < REQUIRED_GOOD_UPDATES) {
            SmartDashboard.putString("Vision/Status", "Pose too different");
            consecutiveBadUpdates++;
            return;
        }
        
        // Calculate appropriate standard deviations based on quality
        double[] stdDevs = calculateStdDevs(poseEstimate, poseDifference);
        
        // Add vision measurement to pose estimator
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
        swerveDrive.addVisionMeasurement(estimatedPose, poseEstimate.timestampSeconds);
        
        // Update tracking
        consecutiveGoodUpdates++;
        consecutiveBadUpdates = 0;
        
        // Publish for visualization
        visionPosePublisher.set(estimatedPose);
        
        SmartDashboard.putString("Vision/Status", "Active");
    }
    
    /**
     * Validates a pose estimate based on various quality metrics
     */
    private boolean isValidPoseEstimate(PoseEstimate poseEstimate, LimelightResults results) {
        // Must have at least one tag
        if (poseEstimate.tagCount == 0) {
            SmartDashboard.putString("Vision/Status", "No tags detected");
            return false;
        }
        
        // Check results validity
        if (!results.valid) {
            SmartDashboard.putString("Vision/Status", "Invalid results");
            return false;
        }
        
        // Check average tag ambiguity
        double avgAmbiguity = poseEstimate.getAvgTagAmbiguity();
        if (avgAmbiguity > MAX_AMBIGUITY) {
            SmartDashboard.putString("Vision/Status", "High ambiguity: " + String.format("%.2f", avgAmbiguity));
            return false;
        }
        
        // Check if pose is reasonable (not off the field)
        Pose3d pose = poseEstimate.pose;
        if (pose.getX() < -1 || pose.getX() > 18 || 
            pose.getY() < -1 || pose.getY() > 9 ||
            Math.abs(pose.getZ()) > 0.5) {
            SmartDashboard.putString("Vision/Status", "Pose out of bounds");
            return false;
        }
        
        return true;
    }
    
    /**
     * Adjusts pose estimate for alliance color
     */
    private Pose2d getAllianceAdjustedPose(PoseEstimate poseEstimate) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        // Default to blue alliance if not connected
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            // Red alliance - flip coordinates
            return new Pose2d(
                16.54 - poseEstimate.pose.getX(),
                8.02 - poseEstimate.pose.getY(),
                poseEstimate.pose.toPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))
            );
        } else {
            // Blue alliance - use as is
            return poseEstimate.pose.toPose2d();
        }
    }
    
    /**
     * Calculates appropriate standard deviations based on measurement quality
     */
    private double[] calculateStdDevs(PoseEstimate poseEstimate, double poseDifference) {
        // Multiple tags = higher confidence
        if (poseEstimate.tagCount >= 2) {
            // Very close to odometry with multiple tags = very high confidence
            if (poseDifference < 0.3 && consecutiveGoodUpdates > REQUIRED_GOOD_UPDATES) {
                return HIGH_CONFIDENCE_STD_DEVS;
            }
            return MULTI_TAG_STD_DEVS;
        }
        
        // Single tag = lower confidence
        return SINGLE_TAG_STD_DEVS;
    }
    
    /**
     * Updates telemetry information on SmartDashboard
     */
    private void updateTelemetry() {
        Optional<PoseEstimate> poseEstimateOpt = poseEstimator.getPoseEstimate();
        
        SmartDashboard.putBoolean("Vision/HasTarget", poseEstimateOpt.isPresent());
        SmartDashboard.putNumber("Vision/ConsecutiveGoodUpdates", consecutiveGoodUpdates);
        SmartDashboard.putNumber("Vision/ConsecutiveBadUpdates", consecutiveBadUpdates);
        
        if (poseEstimateOpt.isPresent()) {
            PoseEstimate poseEstimate = poseEstimateOpt.get();
            SmartDashboard.putNumber("Vision/TagCount", poseEstimate.tagCount);
            SmartDashboard.putNumber("Vision/AvgAmbiguity", poseEstimate.getAvgTagAmbiguity());
            SmartDashboard.putNumber("Vision/AvgDistance", poseEstimate.avgTagDist);
            SmartDashboard.putNumber("Vision/AvgArea", poseEstimate.avgTagArea);
            
            Pose2d estimatedPose = getAllianceAdjustedPose(poseEstimate);
            SmartDashboard.putNumber("Vision/EstimatedX", estimatedPose.getX());
            SmartDashboard.putNumber("Vision/EstimatedY", estimatedPose.getY());
            SmartDashboard.putNumber("Vision/EstimatedRotation", estimatedPose.getRotation().getDegrees());
        }
    }
    
    /**
     * Enables or disables vision updates
     */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
        SmartDashboard.putBoolean("Vision/Enabled", enabled);
    }
    
    /**
     * Gets whether vision is currently enabled
     */
    public boolean isVisionEnabled() {
        return visionEnabled;
    }
    
    /**
     * Gets the Limelight instance for advanced usage
     */
    public Limelight getLimelight() {
        return limelight;
    }
    
    /**
     * Resets vision tracking (useful after teleop init or large odometry changes)
     */
    public void resetTracking() {
        consecutiveGoodUpdates = 0;
        consecutiveBadUpdates = 0;
    }
    
    /**
     * Forces the LED mode on the Limelight
     */
    public void setLEDMode(LEDMode mode) {
        limelight.getSettings()
                .withLimelightLEDMode(mode)
                .save();
    }
    
    /**
     * Gets whether vision has a valid target
     */
    public boolean hasTarget() {
        Optional<LimelightResults> results = limelight.getLatestResults();
        return results.isPresent() && results.get().valid;
    }
}
