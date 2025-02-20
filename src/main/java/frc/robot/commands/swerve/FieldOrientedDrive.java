package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FieldOrientedDrive extends Command{
    
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingAdjust;
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading = false;

    public FieldOrientedDrive(SwerveSubsystem swerve, 
                              DoubleSupplier vX,
                              DoubleSupplier vY,
                              DoubleSupplier headingAdjust,
                              BooleanSupplier lookAway,
                              BooleanSupplier lookTowards,
                              BooleanSupplier lookLeft,
                              BooleanSupplier lookRight) {

                                this.swerve = swerve;
                                this.vX = vX;
                                this.vY = vY;
                                this.headingAdjust = headingAdjust;
                                this.lookAway = lookAway;
                                this.lookTowards = lookTowards;
                                this.lookLeft = lookLeft;
                                this.lookRight = lookRight;

                                addRequirements(swerve);

                              }

                              @Override
                              public void initialize() {
                                resetHeading = true;
                              }

                              @Override
                              public void execute() {
                                double headingX = 0;
                                double headingY = 0;

                                //These allow for 45 degree angle combinations

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
                                        Math.abs(headingAdjust.getAsDouble()) == 0) {
                                            Rotation2d currentHeading = swerve.getHeading();

                                            headingX = currentHeading.getCos();
                                            headingY = currentHeading.getSin();
                                        }
                                        //dont reset heading again
                                        resetHeading = false;
                                }

                                ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

                              }
}
