package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.combinations.HumanPlayerStationCommand;
import frc.robot.commands.combinations.L1Command;
import frc.robot.commands.combinations.L2Command;
import frc.robot.commands.combinations.L3Command;
import frc.robot.commands.combinations.L4Command;
import frc.robot.commands.combinations.RestCommand;
import frc.robot.commands.mechanisms.elevator.ElevatorDataCommand;
import frc.robot.commands.mechanisms.elevator.ElevatorTestCommand;
import frc.robot.commands.mechanisms.shooter.ShooterInitCommand;
import frc.robot.commands.mechanisms.shooter.ShooterIntakeCommand;
import frc.robot.commands.mechanisms.shooter.ShooterL24Command;
import frc.robot.commands.swerve.FieldOrientedPOVDrive;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve")
    );
    
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    // Controllers
    private final CommandXboxController driverXbox =
        new CommandXboxController(DriverConstants.kDriverControllerPort);
    private final CommandXboxController operatorXbox =
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Commands
    private final ElevatorTestCommand elevatorTestCommand = 
        new ElevatorTestCommand(elevatorSubsystem); 
    private final ShooterInitCommand shooterInitCommand = 
        new ShooterInitCommand(shooterSubsystem);
    private final ElevatorDataCommand elevatorDataCommand = 
        new ElevatorDataCommand(elevatorSubsystem);
    private final L1Command l1Command = new L1Command(elevatorSubsystem);
    private final L2Command l2Command = new L2Command(elevatorSubsystem);
    private final L3Command l3Command = new L3Command(elevatorSubsystem);
    private final L4Command l4Command = new L4Command(elevatorSubsystem);
    private final RestCommand restCommand = new RestCommand(elevatorSubsystem);
    private final HumanPlayerStationCommand humanPlayerStationCommand = 
        new HumanPlayerStationCommand(elevatorSubsystem);

    // Swerve input streams
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY() * 1,
        () -> driverXbox.getLeftX() * 1
    )
    .withControllerRotationAxis(driverXbox::getRightX)
    .deadband(DriverConstants.DEADBAND)
    .scaleTranslation(1)
    .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(
            () -> driverXbox.getRightX(),
            () -> driverXbox.getRightY()
        )
        .headingWhile(true);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        elevatorSubsystem.setDefaultCommand(elevatorDataCommand);
    }

    private void configureBindings() {
        // Elevator height commands
        driverXbox.a().whileTrue(l1Command);
        driverXbox.b().whileTrue(l2Command);
        driverXbox.x().whileTrue(l3Command);
        driverXbox.y().whileTrue(l4Command);
        driverXbox.rightTrigger().whileTrue(humanPlayerStationCommand);
        driverXbox.rightBumper().whileTrue(restCommand);

        // Shooter commands
        driverXbox.leftBumper().whileTrue(new ShooterIntakeCommand(shooterSubsystem));
        driverXbox.leftTrigger().whileTrue(new ShooterL24Command(shooterSubsystem));

        // Swerve drive configuration
        FieldOrientedPOVDrive fieldOrientedPOVDrive = new FieldOrientedPOVDrive(
            drivebase, 
            () -> driverXbox.povUp().getAsBoolean(),
            () -> driverXbox.povRight().getAsBoolean(),
            () -> driverXbox.povLeft().getAsBoolean(),
            () -> driverXbox.povDown().getAsBoolean(),
            () -> driverXbox.povUpRight().getAsBoolean(),
            () -> driverXbox.povDownRight().getAsBoolean(),
            () -> driverXbox.povDownLeft().getAsBoolean(),
            () -> driverXbox.povUpLeft().getAsBoolean(),
            () -> driverXbox.getRightX(),
            () -> driverXbox.getRightY(),
            driveDirectAngle
        );

        drivebase.setDefaultCommand(fieldOrientedPOVDrive);

        // FULL SYSTEM RESET - Use when orientation is lost (Start + Back together)
        new Trigger(() -> driverXbox.start().getAsBoolean() && driverXbox.back().getAsBoolean())
            .onTrue(new InstantCommand(() -> {
                drivebase.fullSystemReset();
                SmartDashboard.putString("Driver/Alert", "FULL RESET COMPLETE!");
            }));
        
        // Quick gyro zero (Start button only)
        driverXbox.start().and(driverXbox.back().negate()).onTrue(
            new InstantCommand(() -> {
                drivebase.zeroGyro();
                SmartDashboard.putString("Driver/Alert", "Gyro Zeroed");
            })
        );
        
        // Vision enable/disable toggle (back button only)
        driverXbox.back().and(driverXbox.start().negate()).onTrue(
            new InstantCommand(() -> {
                boolean currentState = drivebase.getVisionSubsystem().isVisionEnabled();
                drivebase.setVisionEnabled(!currentState);
                SmartDashboard.putString("Driver/Alert", 
                    currentState ? "Vision OFF" : "Vision ON");
            })
        );
        
        // Manual encoder sync (Left stick click)
        driverXbox.leftStick().onTrue(new InstantCommand(() -> {
            drivebase.synchronizeModuleEncoders();
            SmartDashboard.putString("Driver/Alert", "Encoders Synced");
        }));
    }

    public Command shooterInitCommand() {
        return new ShooterInitCommand(shooterSubsystem);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    /**
     * Gets the swerve subsystem for direct access
     */
    public SwerveSubsystem getSwerveSubsystem() {
        return drivebase;
    }
}
