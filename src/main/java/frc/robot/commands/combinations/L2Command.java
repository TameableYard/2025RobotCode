package frc.robot.commands.combinations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.PivotSubsystem;

public class L2Command extends Command {

    //private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

    private final ElevatorSubsystem elevatorSubsystem;
    private final PivotSubsystem pivotSubsystem;
    public L2Command(ElevatorSubsystem eSubsystem, PivotSubsystem pSubsystem) {
        elevatorSubsystem = eSubsystem;
        pivotSubsystem = pSubsystem;
        addRequirements(elevatorSubsystem, pivotSubsystem);
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

        SmartDashboard.putNumber("throughborePos: ", pivotSubsystem.getThroughborePos());
        SmartDashboard.putNumber("motor pos: ", pivotSubsystem.getMotorPos());
        SmartDashboard.putNumber("throughborePosRads: ", pivotSubsystem.getThroughborePosRadians());
        
        elevatorSubsystem.reachGoal(ElevatorConstants.kL2Height);
        

        
    }

    @Override
    public void execute() {
        //System.out.println("position error: " + m_ArmSubsystem.showPositionError());
        
        //elevatorSubsystem.synchronizeEncoders();
        
        SmartDashboard.putNumber("elevatorHeight: ", elevatorSubsystem.getHeightMeters());
        SmartDashboard.putNumber("elevatorHeightFrontEncoder: ", elevatorSubsystem.getHeightMetersFrontEncoder());

        SmartDashboard.putNumber("frontMCAppliedOutput: ", elevatorSubsystem.frontMCAppliedOutput());
        SmartDashboard.putNumber("backMCAppliedOutput: ", elevatorSubsystem.backMCAppliedOutput());

        SmartDashboard.putNumber("throughborePos: ", pivotSubsystem.getThroughborePos());
        SmartDashboard.putNumber("motor pos: ", pivotSubsystem.getMotorPos());
        SmartDashboard.putNumber("throughborePosRads: ", pivotSubsystem.getThroughborePosRadians());

        elevatorSubsystem.reachGoal(ElevatorConstants.kL2Height);


        if (elevatorSubsystem.getHeightMeters() > ElevatorConstants.kSafetyHeight) {
            pivotSubsystem.reachSetpoint(PivotConstants.kScoreRot);
        } else {
            pivotSubsystem.noSetpoint();
        }
        //pivotSubsystem.reachSetpoint(0.48); //0.48 vertical, 0.386 com at 0 rad
    }
 
    @Override
    public void end(boolean interrupted) {
        //pivotSubsystem.noSetpoint();
        //elevatorSubsystem.stopMotors();
    }
    
}