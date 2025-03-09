package frc.robot.commands.mechanisms.pivot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.PivotSubsystem;

public class PivotTestCommand extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

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
        SmartDashboard.putNumber("throughborePos: ", pivotSubsystem.getThroughborePos());
        SmartDashboard.putNumber("motor pos: ", pivotSubsystem.getMotorPos());
        

        
    }

    @Override
    public void execute() {
        //System.out.println("position error: " + m_ArmSubsystem.showPositionError());
        System.out.println("throughborePos: " + pivotSubsystem.getThroughborePos());
        System.out.println("motor pos: " + pivotSubsystem.getMotorPos());
        pivotSubsystem.synchronizeEncoders();
        
        SmartDashboard.putNumber("throughborePos: ", pivotSubsystem.getThroughborePos());
        SmartDashboard.putNumber("motor pos: ", pivotSubsystem.getMotorPos());
    }
    
}