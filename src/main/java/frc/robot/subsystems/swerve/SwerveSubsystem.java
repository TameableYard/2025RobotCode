package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final VisionSubsystem visionSubsystem;
    private boolean blueAlliance;
    private Pose2d startingPose;
    
    // Track if we've done initial sync
    private boolean hasInitializedModules = false;
    private final Timer initTimer = new Timer();
    private static final double INIT_DELAY = 0.5; // Wait 500ms for encoders to stabilize
    
    // Track last known good gyro angle for collision detection
    private Rotation2d lastGyroAngle = new Rotation2d();
    private final Timer gyroCheckTimer = new Timer();
    private static final double GYRO_CHECK_INTERVAL = 0.02; // 20ms
    private static final double MAX_GYRO_JUMP = Math.toRadians(10); // 10 degrees per cycle
    
    StructPublisher<Pose2d> odometryPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Odometry/RobotPose", Pose2d.struct).publish();

    public SwerveSubsystem(File directory) {
        // Determine alliance and starting pose
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            blueAlliance = false;
        } else {
            blueAlliance = true;
        }

        startingPose = blueAlliance ? 
            new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)) :
            new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                SwerveConstants.MAX_SPEED, 
                startingPose
            );
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        // Configure swerve drive
        swerveDrive.setHeadingCorrection(false);
        
        if (SwerveDriveTelemetry.isSimulation) {
            swerveDrive.setCosineCompensator(false);
        } else {
            swerveDrive.setCosineCompensator(true);
        }

        // CRITICAL: Enable auto-sync with longer delay for CANCoders to stabilize
        // This ensures absolute encoders are read and applied every loop
        swerveDrive.setModuleEncoderAutoSynchronize(true, 5); // Sync every 5 loops instead of 1
        
        // Push offsets to make sure they're applied
        swerveDrive.pushOffsetsToEncoders();
        
        // Initialize vision subsystem
        visionSubsystem = new VisionSubsystem(swerveDrive);
        
        // Stop default odometry thread since vision subsystem will manage updates
        swerveDrive.stopOdometryThread();
        
        // Start initialization timer
        initTimer.start();
        gyroCheckTimer.start();
        
        // Setup PathPlanner
        setupPathPlanner();
        
        SmartDashboard.putBoolean("Swerve/ModulesInitialized", false);
    }

    @Override
    public void periodic() {
        // Handle module initialization with delay
        if (!hasInitializedModules && initTimer.hasElapsed(INIT_DELAY)) {
            synchronizeModuleEncoders();
            hasInitializedModules = true;
            SmartDashboard.putBoolean("Swerve/ModulesInitialized", true);
        }
        
        // Check for gyro anomalies (possible collision)
        if (gyroCheckTimer.hasElapsed(GYRO_CHECK_INTERVAL)) {
            checkGyroForAnomalies();
            gyroCheckTimer.reset();
        }
        
        // Update odometry manually
        swerveDrive.updateOdometry();
        
        // Publish pose for visualization
        odometryPublisher.set(swerveDrive.getPose());
        
        // Update telemetry
        updateTelemetry();
    }
    
    /**
     * Synchronizes all module encoders with their absolute positions
     * Call this at startup and after collisions
     */
    public void synchronizeModuleEncoders() {
        SmartDashboard.putString("Swerve/Status", "Syncing encoders...");
        
        // Force immediate synchronization of all modules
        swerveDrive.synchronizeModuleEncoders();
        
        // Log the sync
        SmartDashboard.putNumber("Swerve/LastSyncTime", Timer.getFPGATimestamp());
        SmartDashboard.putString("Swerve/Status", "Encoders synced!");
        
        // Small delay to let sync complete
        Timer.delay(0.1);
    }
    
    /**
     * Checks for sudden gyro angle changes that might indicate a collision
     * or encoder desynchronization
     */
    private void checkGyroForAnomalies() {
        Rotation2d currentGyroAngle = swerveDrive.getYaw();
        double angleDifference = Math.abs(
            currentGyroAngle.minus(lastGyroAngle).getRadians()
        );
        
        // If we detect a large sudden change, it might be a collision
        if (angleDifference > MAX_GYRO_JUMP) {
            SmartDashboard.putBoolean("Swerve/PossibleCollisionDetected", true);
            SmartDashboard.putNumber("Swerve/GyroJump", Math.toDegrees(angleDifference));
            
            // Auto-resync modules after potential collision
            synchronizeModuleEncoders();
        } else {
            SmartDashboard.putBoolean("Swerve/PossibleCollisionDetected", false);
        }
        
        lastGyroAngle = currentGyroAngle;
    }
    
    /**
     * Updates telemetry information on SmartDashboard
     */
    private void updateTelemetry() {
        Pose2d pose = swerveDrive.getPose();
        
        SmartDashboard.putNumber("Swerve/RobotX", pose.getX());
        SmartDashboard.putNumber("Swerve/RobotY", pose.getY());
        SmartDashboard.putNumber("Swerve/RobotRotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Swerve/GyroYaw", swerveDrive.getYaw().getDegrees());
        SmartDashboard.putNumber("Swerve/GyroPitch", swerveDrive.getPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/GyroRoll", swerveDrive.getRoll().getDegrees());
        
        // Show individual module angles for debugging
        var modules = swerveDrive.getModules();
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Swerve/Module" + i + "/Angle", 
                modules[i].getAbsolutePosition());
            SmartDashboard.putNumber("Swerve/Module" + i + "/Velocity", 
                modules[i].getDriveVelocity());
        }
    }

    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            
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

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    private static double adjustSensitivity(double value) {
        double sign = Math.signum(value);
        double absvalue = Math.abs(value);
        value = sign * Math.pow(absvalue, Constants.DriverConstants.kSensitivity);
        return value;
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = adjustSensitivity(xInput);
        yInput = adjustSensitivity(yInput);
        return swerveDrive.swerveController.getTargetSpeeds(
            xInput, yInput, angle.getRadians(), 
            getHeading().getRadians(), SwerveConstants.MAX_SPEED
        );
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

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }
    
    /**
     * Zeros the gyro to current heading
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
        lastGyroAngle = new Rotation2d(); // Reset tracking
        SmartDashboard.putString("Swerve/Status", "Gyro zeroed");
    }
    
    /**
     * Resets odometry and re-synchronizes all encoders
     * Use this at match start or after significant impacts
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        // First synchronize encoders
        synchronizeModuleEncoders();
        
        // Then reset odometry
        swerveDrive.resetOdometry(initialHolonomicPose);
        
        // Reset vision tracking when odometry is reset
        visionSubsystem.resetTracking();
        
        // Reset gyro tracking
        lastGyroAngle = initialHolonomicPose.getRotation();
        
        SmartDashboard.putString("Swerve/Status", "Odometry reset");
    }
    
    /**
     * Full system reset - use when robot orientation is completely lost
     * This is what should be called by your "zero" button
     */
    public void fullSystemReset() {
        SmartDashboard.putString("Swerve/Status", "Full reset in progress...");
        
        // 1. Synchronize all module encoders first
        synchronizeModuleEncoders();
        
        // 2. Zero the gyro
        swerveDrive.zeroGyro();
        
        // 3. Reset odometry to current position (or known start position)
        // Using current gyro angle (now zeroed) and current position estimate
        Pose2d currentPose = new Pose2d(
            swerveDrive.getPose().getTranslation(), 
            new Rotation2d() // Gyro is now zero
        );
        swerveDrive.resetOdometry(currentPose);
        
        // 4. Reset vision tracking
        visionSubsystem.resetTracking();
        
        // 5. Reset tracking variables
        lastGyroAngle = new Rotation2d();
        hasInitializedModules = true;
        
        // 6. Log the reset
        SmartDashboard.putNumber("Swerve/LastFullResetTime", Timer.getFPGATimestamp());
        SmartDashboard.putString("Swerve/Status", "Full reset complete!");
        
        Timer.delay(0.2); // Brief delay to ensure everything settles
    }
    
    /**
     * Gets the vision subsystem
     */
    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
    
    /**
     * Enables or disables vision updates
     */
    public void setVisionEnabled(boolean enabled) {
        visionSubsystem.setVisionEnabled(enabled);
    }
    
    /**
     * Returns true if modules have been initialized
     */
    public boolean areModulesInitialized() {
        return hasInitializedModules;
    }
}
// Add these to your SwerveSubsystem.java

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ADD THESE FIELDS TO SwerveSubsystem CLASS
 */
private Pigeon2 pigeon2;
private final Timer pigeonHealthTimer = new Timer();
private static final double HEALTH_CHECK_INTERVAL = 1.0; // Check every second
private int pigeonErrorCount = 0;
private int pigeonReadAttempts = 0;
private double lastValidYaw = 0;
private boolean pigeonHealthy = true;

/**
 * ADD THIS TO YOUR CONSTRUCTOR (after swerveDrive is created)
 */
private void initializePigeonMonitoring() {
    // Get Pigeon reference
    pigeon2 = (Pigeon2) swerveDrive.getGyro().getIMU();
    
    // Verify Pigeon exists and is communicating
    if (pigeon2 == null) {
        SmartDashboard.putString("Pigeon/Status", "ERROR: Not found!");
        pigeonHealthy = false;
        return;
    }
    
    // Check firmware version
    String version = pigeon2.getVersion().toString();
    SmartDashboard.putString("Pigeon/FirmwareVersion", version);
    
    // Get and display configuration
    Pigeon2Configuration config = new Pigeon2Configuration();
    StatusCode status = pigeon2.getConfigurator().refresh(config);
    
    if (status.isOK()) {
        SmartDashboard.putString("Pigeon/Status", "Initialized");
        SmartDashboard.putNumber("Pigeon/MountYaw", config.MountPose.MountPoseYaw);
        SmartDashboard.putNumber("Pigeon/MountPitch", config.MountPose.MountPosePitch);
        SmartDashboard.putNumber("Pigeon/MountRoll", config.MountPose.MountPoseRoll);
    } else {
        SmartDashboard.putString("Pigeon/Status", "Config read failed: " + status.getName());
        pigeonHealthy = false;
    }
    
    // Check initial readings
    double initialYaw = pigeon2.getYaw().getValueAsDouble();
    if (Double.isNaN(initialYaw)) {
        SmartDashboard.putString("Pigeon/Status", "ERROR: Invalid readings!");
        pigeonHealthy = false;
    } else {
        lastValidYaw = initialYaw;
        SmartDashboard.putString("Pigeon/Status", "Healthy");
    }
    
    pigeonHealthTimer.start();
}

/**
 * ADD THIS TO YOUR periodic() METHOD
 */
private void monitorPigeonHealth() {
    if (!pigeonHealthTimer.hasElapsed(HEALTH_CHECK_INTERVAL)) {
        return;
    }
    
    pigeonHealthTimer.reset();
    pigeonReadAttempts++;
    
    // Try to read yaw
    double currentYaw = pigeon2.getYaw().getValueAsDouble();
    
    // Check for invalid reading
    if (Double.isNaN(currentYaw)) {
        pigeonErrorCount++;
        pigeonHealthy = false;
        SmartDashboard.putString("Pigeon/Status", "ERROR: Invalid reading!");
    } else {
        // Check for unreasonable jump (possible desync)
        double yawDelta = Math.abs(currentYaw - lastValidYaw);
        
        // Normalize to 0-180 range (account for wraparound)
        if (yawDelta > 180) {
            yawDelta = 360 - yawDelta;
        }
        
        // If we jumped more than 30 degrees in 1 second while stationary, something's wrong
        ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
        double rotationSpeed = Math.abs(speeds.omegaRadiansPerSecond);
        
        if (yawDelta > 30 && rotationSpeed < 0.1) {
            SmartDashboard.putBoolean("Pigeon/SuspiciousJump", true);
            SmartDashboard.putNumber("Pigeon/JumpAmount", yawDelta);
            
            // Auto-resync if we detect this
            synchronizeModuleEncoders();
        } else {
            SmartDashboard.putBoolean("Pigeon/SuspiciousJump", false);
        }
        
        lastValidYaw = currentYaw;
    }
    
    // Calculate error rate
    double errorRate = (double) pigeonErrorCount / pigeonReadAttempts * 100.0;
    
    // Update dashboard
    SmartDashboard.putNumber("Pigeon/ErrorCount", pigeonErrorCount);
    SmartDashboard.putNumber("Pigeon/ErrorRate", errorRate);
    SmartDashboard.putBoolean("Pigeon/Healthy", pigeonHealthy && errorRate < 5.0);
    
    // Detailed readings
    SmartDashboard.putNumber("Pigeon/Yaw", currentYaw);
    SmartDashboard.putNumber("Pigeon/Pitch", pigeon2.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon/Roll", pigeon2.getRoll().getValueAsDouble());
    
    // Angular velocities (useful for detecting mounting issues)
    SmartDashboard.putNumber("Pigeon/AngVelX", pigeon2.getAngularVelocityXWorld().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon/AngVelY", pigeon2.getAngularVelocityYWorld().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon/AngVelZ", pigeon2.getAngularVelocityZWorld().getValueAsDouble());
    
    // Temperature (can indicate electrical issues)
    SmartDashboard.putNumber("Pigeon/Temperature", pigeon2.getTemperature().getValueAsDouble());
    
    // Supply voltage (low voltage can cause problems)
    SmartDashboard.putNumber("Pigeon/SupplyVoltage", pigeon2.getSupplyVoltage().getValueAsDouble());
    
    // Check for concerning conditions
    checkPigeonWarnings();
}

/**
 * ADD THIS METHOD
 */
private void checkPigeonWarnings() {
    boolean hasWarning = false;
    StringBuilder warnings = new StringBuilder();
    
    // Check temperature
    double temp = pigeon2.getTemperature().getValueAsDouble();
    if (temp > 70) {
        warnings.append("HIGH TEMP! ");
        hasWarning = true;
    }
    
    // Check voltage
    double voltage = pigeon2.getSupplyVoltage().getValueAsDouble();
    if (voltage < 4.5 || voltage > 5.5) {
        warnings.append("BAD VOLTAGE! ");
        hasWarning = true;
    }
    
    // Check error rate
    double errorRate = (double) pigeonErrorCount / Math.max(1, pigeonReadAttempts) * 100.0;
    if (errorRate > 10) {
        warnings.append("HIGH ERROR RATE! ");
        hasWarning = true;
    }
    
    // Check for extreme pitch/roll (mounting issue?)
    double pitch = Math.abs(pigeon2.getPitch().getValueAsDouble());
    double roll = Math.abs(pigeon2.getRoll().getValueAsDouble());
    
    if (pitch > 15 || roll > 15) {
        warnings.append("TILTED MOUNTING! ");
        hasWarning = true;
    }
    
    if (hasWarning) {
        SmartDashboard.putString("Pigeon/Warnings", warnings.toString());
        SmartDashboard.putBoolean("Pigeon/HasWarnings", true);
    } else {
        SmartDashboard.putString("Pigeon/Warnings", "None");
        SmartDashboard.putBoolean("Pigeon/HasWarnings", false);
    }
}

/**
 * ADD THIS METHOD - Call from RobotContainer for testing
 */
public void performPigeonFactoryReset() {
    SmartDashboard.putString("Pigeon/Status", "Factory reset in progress...");
    
    // Clear accumulated faults
    pigeon2.clearStickyFaults();
    
    // Apply factory default configuration
    Pigeon2Configuration config = new Pigeon2Configuration();
    StatusCode status = pigeon2.getConfigurator().apply(config);
    
    if (status.isOK()) {
        SmartDashboard.putString("Pigeon/Status", "Factory reset complete!");
        
        // Reset our tracking variables
        pigeonErrorCount = 0;
        pigeonReadAttempts = 0;
        pigeonHealthy = true;
        
        // Wait for Pigeon to stabilize
        Timer.delay(1.0);
        
        // Update last valid reading
        lastValidYaw = pigeon2.getYaw().getValueAsDouble();
    } else {
        SmartDashboard.putString("Pigeon/Status", "Factory reset FAILED: " + status.getName());
    }
}

/**
 * ADD THIS METHOD - Returns health status
 */
public boolean isPigeonHealthy() {
    double errorRate = (double) pigeonErrorCount / Math.max(1, pigeonReadAttempts) * 100.0;
    return pigeonHealthy && errorRate < 5.0;
}

/**
 * MODIFY YOUR EXISTING periodic() TO CALL:
 */
@Override
public void periodic() {
    // ... existing code ...
    
    // Add Pigeon monitoring
    monitorPigeonHealth();
    
    // ... rest of existing code ...
}
