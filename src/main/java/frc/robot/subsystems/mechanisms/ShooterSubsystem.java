package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorLimit;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    private final SparkMax shooterMotor = new SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

    private final LaserCan coralSensor = new LaserCan(ShooterConstants.kLaserCANPort);
    

    

    private final RelativeEncoder shooterMotorEncoder = shooterMotor.getEncoder();

    public static double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0);

    //private static final double kFlywheelGearing = 4.0;
    //TODO: get proper MoIs from onshape, including EVERYTHING that rotates
    //private static final double kBigFlywheelMomentOfInertia = .00012916529; //kg * m^2
    //private static final double kSmallFlywheelMomentOfInertia = 4.3228437e-5; //kg * m^2

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_FlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNEO(1), ShooterConstants.kFlywheelMomentOfInertia, ShooterConstants.kFlywheelGearing);


  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_Observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_FlywheelPlant,
          VecBuilder.fill(ShooterConstants.kStateStdDevs), // How accurate we think our model is
          VecBuilder.fill(ShooterConstants.kMeasurementStdDevs), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_Controller =
      new LinearQuadraticRegulator<>(
          m_FlywheelPlant,
          VecBuilder.fill(85.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_Loop =
      new LinearSystemLoop<>(m_FlywheelPlant, m_Controller, m_Observer, ShooterConstants.kMaxVoltage, 0.020);

    public void initFlywheel() {
        m_Loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(shooterMotorEncoder.getVelocity())));
    }

    public boolean isSpunUp() {
        if (Math.abs(shooterMotorEncoder.getVelocity()) > ShooterConstants.kSpunUpRPM) {
            return true;
        }
        else {
            return false;
        }
    }

    public ShooterSubsystem () {
        SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

        shooterMotorConfig.smartCurrentLimit(MotorLimit.Neo.stall, MotorLimit.Neo.free, MotorLimit.Neo.stallRPM);

        shooterMotor.configure(shooterMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*public void unguidedShoot() {
        shooterMotor.set(-1);
    }

    public void guidedShoot(double desiredSpeed) { //pass in desired rpm and convert to radians
        kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(-desiredSpeed);
    }*/

    public void guidedShoot(double desiredSpeed) {
        kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(desiredSpeed);
    }

    public void stop() {
        kSpinupRadPerSec = 0;
    }

    public boolean hasCoral() {
        LaserCan.Measurement measurement = coralSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm < ShooterConstants.kSensorDistance) {
            return true;
        } else {
            return false;
        }
    }


    @Override
    public void periodic() {
        //System.out.println("shooter speed: " + ((shooterEncoder.getVelocity()*(Math.PI*0.1016))/60 )+ " m/s");
    
        m_Loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));

        m_Loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(shooterMotorEncoder.getVelocity())));

        m_Loop.predict(0.020);

        double nextVoltage = m_Loop.getU(0);
        shooterMotor.setVoltage(nextVoltage);

        SmartDashboard.putBoolean("hasCoral", hasCoral());


    
    }

}