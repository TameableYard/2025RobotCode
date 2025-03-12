package frc.robot.commands.mechanisms.pivot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;

public class ShooterInitCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;
    public ShooterInitCommand(ShooterSubsystem subsystem) {
        m_ShooterSubsystem = subsystem;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.initBigFlywheel();
        m_ShooterSubsystem.initSmallFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

   
    
}