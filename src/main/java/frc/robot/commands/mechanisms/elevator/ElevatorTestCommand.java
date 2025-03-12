package frc.robot.commands.mechanisms.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

public class ElevatorTestCommand extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

    private final ElevatorSubsystem elevatorSubsystem;
    public ElevatorTestCommand(ElevatorSubsystem subsystem) {
        elevatorSubsystem = subsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        //pivotSubsystem.setGoal(PivotConstants.kAmpShootPos);
        /*if (!pivotSubsystem.isEnabled()) {
            pivotSubsystem.enable();
        }*/
        SmartDashboard.putNumber("elevatorHeight: ", elevatorSubsystem.getHeightMeters());
        SmartDashboard.putNumber("elevatorHeightFrontEncoder: ", elevatorSubsystem.getHeightMetersFrontEncoder());
        SmartDashboard.putNumber("frontMCAppliedOutput: ", elevatorSubsystem.frontMCAppliedOutput());
        SmartDashboard.putNumber("backMCAppliedOutput: ", elevatorSubsystem.backMCAppliedOutput());
        
        
        

        
    }

    @Override
    public void execute() {
        //System.out.println("position error: " + m_ArmSubsystem.showPositionError());
        
        //elevatorSubsystem.synchronizeEncoders();
        
        SmartDashboard.putNumber("elevatorHeight: ", elevatorSubsystem.getHeightMeters());
        SmartDashboard.putNumber("elevatorHeightFrontEncoder: ", elevatorSubsystem.getHeightMetersFrontEncoder());

        SmartDashboard.putNumber("frontMCAppliedOutput: ", elevatorSubsystem.frontMCAppliedOutput());
        SmartDashboard.putNumber("backMCAppliedOutput: ", elevatorSubsystem.backMCAppliedOutput());

        elevatorSubsystem.reachGoal(0.5);



        //pivotSubsystem.reachSetpoint(0.48); //0.48 vertical, 0.386 com at 0 rad
    }
 
    @Override
    public void end(boolean interrupted) {
        //pivotSubsystem.noSetpoint();
        elevatorSubsystem.stopMotors();
    }
    
}