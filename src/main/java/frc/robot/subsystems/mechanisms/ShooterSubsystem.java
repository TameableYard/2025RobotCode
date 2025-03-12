package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorLimit;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    private final SparkMax bigShooterMotor = new SparkMax(ShooterConstants.kBigShooterMotorPort, MotorType.kBrushless);
    private final SparkMax smallShooterMotor = new SparkMax(ShooterConstants.kSmallShooterMotorPort, MotorType.kBrushless);

    
    

    

    private final RelativeEncoder bigShooterMotorEncoder = bigShooterMotor.getEncoder();
    private final RelativeEncoder smallShooterMotorEncoder = smallShooterMotor.getEncoder();

    public static double kBigSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0);
    public static double kSmallSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0);
    //private static final double kFlywheelGearing = 4.0;
    //TODO: get proper MoIs from onshape, including EVERYTHING that rotates
    //private static final double kBigFlywheelMomentOfInertia = .00012916529; //kg * m^2
    //private static final double kSmallFlywheelMomentOfInertia = 4.3228437e-5; //kg * m^2

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_bigFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNeo550(1), ShooterConstants.kBigFlywheelMomentOfInertia, ShooterConstants.kFlywheelGearing);

  private final LinearSystem<N1, N1, N1> m_smallFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
        DCMotor.getNeo550(1), ShooterConstants.kSmallFlywheelMomentOfInertia, ShooterConstants.kFlywheelGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_bigObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_bigFlywheelPlant,
          VecBuilder.fill(ShooterConstants.kStateStdDevs), // How accurate we think our model is
          VecBuilder.fill(ShooterConstants.kMeasurementStdDevs), // How accurate we think our encoder
          // data is
          0.020);

    // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_smallObserver =
  new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      m_smallFlywheelPlant,
      VecBuilder.fill(ShooterConstants.kStateStdDevs), // How accurate we think our model is
      VecBuilder.fill(ShooterConstants.kMeasurementStdDevs), // How accurate we think our encoder
      // data is
      0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_bigController =
      new LinearQuadraticRegulator<>(
          m_bigFlywheelPlant,
          VecBuilder.fill(80.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_smallController =
      new LinearQuadraticRegulator<>(
          m_smallFlywheelPlant,
          VecBuilder.fill(80.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_bigLoop =
      new LinearSystemLoop<>(m_bigFlywheelPlant, m_bigController, m_bigObserver, ShooterConstants.kMaxVoltage, 0.020);

  private final LinearSystemLoop<N1, N1, N1> m_smallLoop =
      new LinearSystemLoop<>(m_smallFlywheelPlant, m_smallController, m_smallObserver, ShooterConstants.kMaxVoltage, 0.020);


    public void initSmallFlywheel() {
        m_smallLoop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(smallShooterMotorEncoder.getVelocity())));
    }

    public void initBigFlywheel() {
        m_bigLoop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(bigShooterMotorEncoder.getVelocity())));
    }

    public boolean isBigSpunUp() {
        if (Math.abs(bigShooterMotorEncoder.getVelocity()) > ShooterConstants.kBigSpunUpRPM) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean isSmallSpunUp() {
        if (Math.abs(smallShooterMotorEncoder.getVelocity()) > ShooterConstants.kSmallSpunUpRPM) {
            return true;
        }
        else {
            return false;
        }
    }

    public ShooterSubsystem () {
        SparkMaxConfig shooterMotorsConfig = new SparkMaxConfig();
        SparkMaxConfig smallShooterMotorConfig = new SparkMaxConfig();

        shooterMotorsConfig.smartCurrentLimit(MotorLimit.Neo550.stall, MotorLimit.Neo550.free, MotorLimit.Neo550.stallRPM);

        smallShooterMotorConfig.apply(shooterMotorsConfig).inverted(true);

        bigShooterMotor.configure(shooterMotorsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        smallShooterMotor.configure(smallShooterMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*public void unguidedShoot() {
        shooterMotor.set(-1);
    }

    public void guidedShoot(double desiredSpeed) { //pass in desired rpm and convert to radians
        kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(-desiredSpeed);
    }*/

    public void guidedBigShoot(double desiredSpeed) {
        kBigSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(desiredSpeed);
    }

    public void guidedSmallShoot(double desiredSpeed) {
        kSmallSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(desiredSpeed);
    }

    public void stop() {
        kSmallSpinupRadPerSec = 0;
        kBigSpinupRadPerSec = 0;
    }


    @Override
    public void periodic() {
        //System.out.println("shooter speed: " + ((shooterEncoder.getVelocity()*(Math.PI*0.1016))/60 )+ " m/s");
        
        m_smallLoop.setNextR(VecBuilder.fill(kSmallSpinupRadPerSec));

        m_bigLoop.setNextR(VecBuilder.fill(kBigSpinupRadPerSec));

        m_smallLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(smallShooterMotorEncoder.getVelocity())));

        m_bigLoop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(bigShooterMotorEncoder.getVelocity())));

        m_smallLoop.predict(0.020);

        m_bigLoop.predict(0.020);

        double nextBigVoltage = m_bigLoop.getU(0);
        bigShooterMotor.setVoltage(nextBigVoltage);

        double nextSmallVoltage = m_smallLoop.getU(0);
        smallShooterMotor.setVoltage(nextSmallVoltage);

    
    }

}