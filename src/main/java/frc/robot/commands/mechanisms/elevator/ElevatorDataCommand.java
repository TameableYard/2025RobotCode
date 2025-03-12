package frc.robot.commands.mechanisms.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

public class ElevatorDataCommand extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

    private final ElevatorSubsystem elevatorSubsystem;
    public ElevatorDataCommand(ElevatorSubsystem subsystem) {
        elevatorSubsystem = subsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

        SmartDashboard.putNumber("elevatorHeight: ", elevatorSubsystem.getHeightMeters());
        SmartDashboard.putNumber("elevatorHeightFrontEncoder: ", elevatorSubsystem.getHeightMetersFrontEncoder());
        SmartDashboard.putNumber("frontMCAppliedOutput: ", elevatorSubsystem.frontMCAppliedOutput());
        SmartDashboard.putNumber("backMCAppliedOutput: ", elevatorSubsystem.backMCAppliedOutput());
        
        
        

        
    }

    @Override
    public void execute() {
        
        SmartDashboard.putNumber("elevatorHeight: ", elevatorSubsystem.getHeightMeters());
        SmartDashboard.putNumber("elevatorHeightFrontEncoder: ", elevatorSubsystem.getHeightMetersFrontEncoder());
        SmartDashboard.putNumber("frontMCAppliedOutput: ", elevatorSubsystem.frontMCAppliedOutput());
        SmartDashboard.putNumber("backMCAppliedOutput: ", elevatorSubsystem.backMCAppliedOutput());

    }
 
    @Override
    public void end(boolean interrupted) {
        
    }
    
}