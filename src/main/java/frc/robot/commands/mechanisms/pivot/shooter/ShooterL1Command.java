package frc.robot.commands.mechanisms.pivot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;

public class ShooterL1Command extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;
    //private final CommandXboxController operatorXbox;
    public ShooterL1Command(ShooterSubsystem subsystem/*, CommandXboxController operator*/) {
        m_ShooterSubsystem = subsystem;
        //operatorXbox = operator;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.guidedBigShoot(ShooterConstants.kBigL1Speed);
        m_ShooterSubsystem.guidedSmallShoot(ShooterConstants.kSmallL1Speed);
    }

    @Override
    public void execute() {
        //if (m_ShooterSubsystem.isSpunUp()) {
            //operatorXbox.();
       // }
    }

    @Override
    public void end (boolean interrupted) {
        m_ShooterSubsystem.stop();
    }

    
}