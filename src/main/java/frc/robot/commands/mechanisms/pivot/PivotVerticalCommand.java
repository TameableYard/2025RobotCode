package frc.robot.commands.mechanisms.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.PivotSubsystem;

public class PivotVerticalCommand extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

    private final PivotSubsystem pivotSubsystem;
    public PivotVerticalCommand(PivotSubsystem subsystem) {
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
        SmartDashboard.putNumber("throughborePosRads: ", pivotSubsystem.getThroughborePosRadians());
        //vertical
        pivotSubsystem.reachSetpoint(0.48);

        
    }

    @Override
    public void execute() {
        //System.out.println("position error: " + m_ArmSubsystem.showPositionError());
        System.out.println("throughborePos: " + pivotSubsystem.getThroughborePos());
        System.out.println("motor pos: " + pivotSubsystem.getMotorPos());
        pivotSubsystem.synchronizeEncoders();
        
        SmartDashboard.putNumber("throughborePos: ", pivotSubsystem.getThroughborePos());
        SmartDashboard.putNumber("motor pos: ", pivotSubsystem.getMotorPos());
        SmartDashboard.putNumber("throughborePosRads: ", pivotSubsystem.getThroughborePosRadians());

        //pivotSubsystem.reachSetpoint(0.48); //0.48 vertical, 0.386 com at 0 rad
    }
 
    @Override
    public void end(boolean interrupted) {
        //pivotSubsystem.noSetpoint();
    }
    
}