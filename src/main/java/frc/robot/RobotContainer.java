// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.mechanisms.PivotSubsystem;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.mechanisms.pivot.PivotTestCommand;
import frc.robot.commands.swerve.FieldOrientedDrive;
import frc.robot.commands.swerve.FieldOrientedPOVDrive;
 
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

  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final PivotTestCommand pivotTestCommand = new PivotTestCommand(pivotSubsystem);

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


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
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

    driverXbox.a().whileTrue(pivotTestCommand);

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

    //driverXbox.povUp().or(driverXbox.povUpRight().or(driverXbox.povRight().or(driverXbox.povDownRight().or(driverXbox.povDown().or(driverXbox.povDownLeft().or(driverXbox.povLeft().or(driverXbox.povUpLeft()))))))).whileTrue(fieldOrientedPOVDrive);

    

    //FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive();



    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
       //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  //}
}
