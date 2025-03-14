package frc.robot.commands.mechanisms.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

public class ClimberOutCommand extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

    private final ClimberSubsystem climberSubsystem;
    public ClimberOutCommand(ClimberSubsystem subsystem) {
        climberSubsystem = subsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {

        climberSubsystem.climberOut();
        
        
        

        
    }

    @Override
    public void execute() {
        
        climberSubsystem.climberOut();
    }
 
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.noClimber();
    }


}