package frc.robot.commands.mechanisms.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

public class climberInitCommand extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

    private final ClimberSubsystem climberSubsystem;
    public climberInitCommand(ClimberSubsystem subsystem) {
        climberSubsystem = subsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {

        climberSubsystem.changeThroughbore();
        
        
        

        
    }

    @Override
    public void execute() {
        this.cancel();

    }
 
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}