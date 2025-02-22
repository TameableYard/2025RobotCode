package frc.robot.commands.swerve;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;

public class FieldOrientedDrive extends Command{
    
    private final SwerveSubsystem swerve;
    //private final DoubleSupplier vX, vY;
    //private final DoubleSupplier heading;
    //private final DoubleSupplier headingX;
    //private final DoubleSupplier headingY;

    private final SwerveInputStream inputs;
    //private final Supplier<ChassisSpeeds> velocity;


    public FieldOrientedDrive(SwerveSubsystem swerve,
                                SwerveInputStream inputs
                              
                              //DoubleSupplier heading//,
                              //DoubleSupplier headingX,
                              //DoubleSupplier headingY
                              ) {
                                this.swerve = swerve;
                                this.inputs = inputs;
                                //this.headingX = headingX;
                                //this.headingY = headingY;
                                

                                addRequirements(swerve); 

    }

    @Override
    public void initialize() {
        //resetHeading = true;
        
    }

    @Override
    public void execute() {
        ChassisSpeeds desiredSpeeds = inputs.get();

        //These allow for 45 degree angle combinations
/* 
        if (lookAway.getAsBoolean()) {
            headingY = -1;
        }
        if (lookRight.getAsBoolean()) {
            headingX = 1;
        }
        if (lookLeft.getAsBoolean()) {
            headingX = -1;
        }
        if (lookTowards.getAsBoolean()) {
            headingY = 1;
        }



        if (resetHeading) {
            if (headingX == 0 &&
                headingY == 0 && 
                Math.abs(heading.getAsDouble()) == 0) {
                    Rotation2d currentHeading = swerve.getHeading();

                    headingX = currentHeading.getCos();
                    headingY = currentHeading.getSin();
                }
                //dont reset heading again
                resetHeading = false;
        }*/

        //ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), new Rotation2d(heading.getAsDouble() * Math.PI));
        //ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(inputs.get());

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.SwerveConstants.LOOP_TIME, Constants.SwerveConstants.ROBOT_MASS, List.of(Constants.SwerveConstants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

        }
}
