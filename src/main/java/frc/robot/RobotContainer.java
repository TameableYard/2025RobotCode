// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.subsystems.mechanisms.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
//import frc.robot.subsystems.mechanisms.PivotSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

import frc.robot.commands.mechanisms.elevator.ElevatorDataCommand;
import frc.robot.commands.mechanisms.elevator.ElevatorTestCommand;

import frc.robot.commands.mechanisms.shooter.ShooterInitCommand;
import frc.robot.commands.mechanisms.shooter.ShooterIntakeCommand;
//import frc.robot.commands.mechanisms.pivot.shooter.ShooterTestCommand;
//import frc.robot.subsystems.mechanisms.ElevatorSubsystem.runSysIdRoutine;
import frc.robot.commands.swerve.FieldOrientedDrive;
import frc.robot.commands.swerve.FieldOrientedPOVDrive;

//import frc.robot.commands.mechanisms.climber.ClimberOutCommand;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(DriverConstants.kDriverControllerPort);

  private final CommandXboxController operatorXbox =
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  //private final PivotTestCommand pivotTestCommand = new PivotTestCommand(pivotSubsystem);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final ElevatorTestCommand elevatorTestCommand = new ElevatorTestCommand(elevatorSubsystem); 

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  //private final ClimberOutCommand climberOutCommand = new ClimberOutCommand(climberSubsystem);

  //private final ClimberInCommand climberInCommand = new ClimberInCommand(climberSubsystem);

  //private final climberInitCommand climberInitCommand = new climberInitCommand(climberSubsystem);

  private final ShooterInitCommand shooterInitCommand = new ShooterInitCommand(shooterSubsystem);
  
  //private final ShooterTestCommand shooterTestCommand = new ShooterTestCommand(shooterSubsystem);

  //private final runSysIdRoutine runsysidroutine = new runSysIdRoutine(elevatorSubsystem);

  private final ElevatorDataCommand elevatorDataCommand = new ElevatorDataCommand(elevatorSubsystem);

  //private final PivotDataCommand pivotDataCommand = new PivotDataCommand(pivotSubsystem);

  //private final Pivot40Command pivot40Command = new Pivot40Command(pivotSubsystem);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */ 
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * 1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(DriverConstants.DEADBAND)
                                                            .scaleTranslation(1)// was 0.8
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */

  //private DoubleSupplier headingXSupplier;
  //private DoubleSupplier headingYSupplier;
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRightX(),
                                                                                             () -> driverXbox.getRightY())
                                                           .headingWhile(true);

  SwerveInputStream driveSimpleAuto = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> -0.4,
                                                            () -> 0).allianceRelativeControl(false);
                                                            


  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //NamedCommands.registerCommand("", climberInCommand);
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); //default auto will be Commands.none();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    //pivotSubsystem.setDefaultCommand(pivotTestCommand);
    
    //drivebase.setDefaultCommand(new FieldOrientedDrive(driveAngularVelocity));

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    driverXbox.b().whileTrue(elevatorTestCommand);

    driverXbox.x().whileTrue(elevatorSubsystem.runSysIdRoutine());

    //driverXbox.a().whileTrue(pivot40Command);//pivotTestCommand);

    //driverXbox.y().whileTrue(shooterTestCommand);

    //operatorXbox.a().whileTrue(new L3Command(elevatorSubsystem, pivotSubsystem));
    //operatorXbox.b().whileTrue(new L4Command(elevatorSubsystem, pivotSubsystem));
    //operatorXbox.y().whileTrue(new L1Command(elevatorSubsystem, pivotSubsystem));
    //operatorXbox.x().whileTrue(new L2Command(elevatorSubsystem, pivotSubsystem));
    //operatorXbox.leftTrigger().whileTrue(new HumanPlayerStationCommand(elevatorSubsystem, pivotSubsystem));

    //driverXbox.leftBumper().whileTrue(new ClimberInCommand(climberSubsystem));
    //driverXbox.rightBumper().whileTrue(new ClimberOutCommand(climberSubsystem));
    
    driverXbox.rightTrigger().whileTrue(new ShooterIntakeCommand(shooterSubsystem));
    
    //driverXbox.x().whileTrue(elevatorSubsystem.runSysIdRoutine());

    FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(drivebase, 
                                                                   driveDirectAngle);

    FieldOrientedPOVDrive fieldOrientedPOVDrive = new FieldOrientedPOVDrive(drivebase, 
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
                                                                   driveDirectAngle);

    drivebase.setDefaultCommand(fieldOrientedPOVDrive);

    driverXbox.start().onTrue((new InstantCommand(drivebase::zeroGyro)));

    elevatorSubsystem.setDefaultCommand(elevatorDataCommand);

    //pivotSubsystem.setDefaultCommand(pivotDataCommand);

    //driverXbox.povUp().or(driverXbox.povUpRight().or(driverXbox.povRight().or(driverXbox.povDownRight().or(driverXbox.povDown().or(driverXbox.povDownLeft().or(driverXbox.povLeft().or(driverXbox.povUpLeft()))))))).whileTrue(fieldOrientedPOVDrive);

    

    //FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive();



    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
       //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command shooterInitCommand() {
    return new ShooterInitCommand(shooterSubsystem);
  }
/*
  public Command climberInitCommand() {
    return new climberInitCommand(climberSubsystem);
  }*/

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new FieldOrientedDrive(drivebase, driveSimpleAuto);
    return autoChooser.getSelected();
  }
}
