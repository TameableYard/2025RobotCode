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
