package frc.robot.commands.mechanisms.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.PivotSubsystem;

public class PivotTestCommand extends Command {

    private final PivotSubsystem pivotSubsystem;
    public PivotTestCommand(PivotSubsystem subsystem) {
        pivotSubsystem = subsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        //pivotSubsystem.setGoal(PivotConstants.kAmpShootPos);
        /*if (!pivotSubsystem.isEnabled()) {
            pivotSubsystem.enable();
        }*/
        
    }

    @Override
    public void execute() {
        //System.out.println("position error: " + m_ArmSubsystem.showPositionError());
        System.out.println("throughborePos: " + pivotSubsystem.getThroughborePos());
        System.out.println("motor pos: " + pivotSubsystem.getMotorPos());
    }
    
}