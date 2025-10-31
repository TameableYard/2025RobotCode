# Robot Documentation - 2025 Reefscape

## 🛠️ System Overview

### Swerve Drive System
- **Type:** YAGSL-based swerve drive
- **Modules:** 4x MK4i modules
- **Controllers:** SparkMax with NEO motors
- **Encoders:** CANCoders (absolute position)
- **Gyro:** Pigeon 2.0 (CAN ID: 2)
- **Features:**
  - Auto encoder synchronization
  - Collision detection and recovery
  - Three-level reset system
  - Real-time health monitoring

### Vision System
- **Camera:** Limelight 2+
- **Function:** AprilTag detection for pose estimation
- **Integration:** Fuses with swerve odometry
- **Features:**
  - Multi-tag tracking (MegaTag2)
  - Quality filtering
  - Alliance-aware coordinate flipping
  - Toggle-able during match

### Control Systems
- **Driver Controls:** Xbox controller (Port 0)
- **Operator Controls:** Xbox controller (Port 1)
- **Reset Levels:**
  - L3: Quick encoder sync
  - Start: Zero gyro
  - Start+Back: Full system reset

## 📊 Dashboard Layout

### Critical Values to Monitor

#### Swerve Status
```
Swerve/ModulesInitialized: true/false
Swerve/Status: Current operation
Swerve/PossibleCollisionDetected: true/false
```

#### Pigeon Health
```
Pigeon/Healthy: true/false
Pigeon/ErrorRate: < 5% is good
Pigeon/Warnings: Any active issues
```

#### Vision Status
```
Vision/HasTarget: Sees AprilTags
Vision/Status: Active/Inactive
Vision/TagCount: Number of tags visible
```

## 📖 Documentation Maintenance

### When to Update
- After hardware changes
- After significant code changes
- When new issues are discovered
- After competition (lessons learned)

### What to Document
- Configuration changes
- Calibration values
- Hardware modifications
- Known issues and workarounds
- Match performance notes

### How to Contribute
1. Create feature branch
2. Update relevant documentation
3. Submit pull request
4. Get review from team lead

## 📞 Support Resources

### External
- YAGSL Documentation: [GitHub link]
- Limelight Docs: docs.limelightvision.io
- CTRE Phoenix Tuner: support.ctr-electronics.com
- Chief Delphi: chiefdelphi.com

## 🎓 Training Resources

### For New Programmers
1. Read [Vision System Guide](vision/VisionSystemGuide.md)
2. Review [Swerve Troubleshooting](swerve/SwerveTroubleshooting.md)
3. Practice with diagnostic commands
4. Shadow experienced programmer at practice

### For New Drivers
1. Print [Driver Quick Reference](driver/DriverQuickReference.html)
2. Practice in simulator if available
3. Learn all three reset levels
4. Start with simple driving, progress to complex maneuvers

## 📝 Change Log

### 2025-01-31 - Major Swerve Fixes
- Added automatic encoder synchronization
- Implemented collision detection
- Added Pigeon health monitoring
- Created three-level reset system
- Added comprehensive documentation

### 2025-01-31 - Vision System Integration
- Integrated Limelight AprilTag detection
- Added vision pose estimation
- Created filtering and quality checks
- Added toggle controls for drivers

---

**Last Updated:** 10/32/25  
**Maintained By:** FRC Team 1474 Programming Team  
**Season:** 2025 Reefscape

---
