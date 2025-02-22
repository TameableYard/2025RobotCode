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
    private final BooleanSupplier lookAway;
    private final BooleanSupplier lookRight;
    private final BooleanSupplier lookLeft;
    private final BooleanSupplier lookTowards;
    private final BooleanSupplier lookAwayRight;
    private final BooleanSupplier lookTowardsRight;
    private final BooleanSupplier lookTowardsLeft;
    private final BooleanSupplier lookAwayLeft;

    private SwerveInputStream inputs;
    
        
        //private final Supplier<ChassisSpeeds> velocity;
        
    
    
        public FieldOrientedDrive(SwerveSubsystem swerve,
                                  BooleanSupplier lookAway,
                                  BooleanSupplier lookRight,
                                  BooleanSupplier lookLeft,
                                  BooleanSupplier lookTowards,
                                  BooleanSupplier lookAwayRight,
                                  BooleanSupplier lookTowardsRight,
                                  BooleanSupplier lookTowardsLeft,
                                  BooleanSupplier lookAwayLeft,
                                  SwerveInputStream inputs
                                  
                                  //DoubleSupplier heading//,
                                  //DoubleSupplier headingX,
                                  //DoubleSupplier headingY
                                  ) {
                                    this.swerve = swerve;
                                    this.lookAway = lookAway;
                                    this.lookRight = lookRight;
                                    this.lookLeft = lookLeft;
                                    this.lookTowards = lookTowards;
                                    this.lookAwayRight = lookAwayRight;
                                    this.lookTowardsRight = lookTowardsRight;
                                    this.lookTowardsLeft = lookTowardsLeft;
                                    this.lookAwayLeft = lookAwayLeft;
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
    
            //double headingX = 0;
            //DoubleSupplier headingXSupplier;
            //double headingY = 0;
            //DoubleSupplier headingYSupplier;
            
            
    
            //These allow for 45 degree angle combinations
            
     
            if (lookAway.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> 0, () -> -1);
            }
            if (lookRight.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> 1, () -> 0);
            }
            if (lookLeft.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> -1, () -> 0);
            }
            if (lookTowards.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> 0, () -> 1);
            }
            if (lookAwayRight.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> 1, () -> -1);
            }
            if (lookTowardsRight.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> 1, () -> 1);
            }
            if (lookTowardsLeft.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> -1, () -> 1);
            }
            if (lookAwayLeft.getAsBoolean()) {
                inputs = inputs.withControllerHeadingAxis(() -> -1, () -> -1);
            }

        //headingXSupplier = () -> headingX;
        //headingYSupplier = () -> headingY;

        //inputs.withControllerHeadingAxis(headingXSupplier, headingYSupplier);
        
        ChassisSpeeds desiredSpeeds = inputs.get();


        /*if (resetHeading) {
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
