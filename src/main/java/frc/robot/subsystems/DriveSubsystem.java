package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleopDriveCommand;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;
import org.strykeforce.thirdcoast.telemetry.item.TalonItem;

public class DriveSubsystem extends SubsystemBase {

  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;
  private static final double MAX_VELOCITY = 1000;
  private static final double MAX_ACCELERATION = 1.0;
  private static final double MAX_JERK = 1.0;
  private static final int TICKS_PER_REV = 4096;
  private static final double WHEEL_DIAMETER = 0.0635; // In meters
  private static final double TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
  private static final double kP_PATH = 0.1;
  private static final double kV_PATH = 1 / MAX_VELOCITY;

  private final SwerveDrive swerve = configSwerve();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  private EncoderFollower follower = new EncoderFollower();
  private Trajectory trajectoryGenerated;
  private int talonPosition;
  private int segments;

  public DriveSubsystem() {
    swerve.setFieldOriented(true);
    setDefaultCommand(new TeleopDriveCommand());
  }

  public void drive(double forward, double strafe, double yaw) {
    swerve.drive(forward, strafe, yaw);
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = gyro.getAngle() % 360;
    gyro.setAngleAdjustment(-adj);
    logger.info("resetting gyro: ({})", adj);
  }

  private Wheel[] getWheels() {
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;
    azimuthConfig.slot0.kP = 10.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 100.0;
    azimuthConfig.slot0.kF = 0.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.velocityMeasurementWindow = 64;
    azimuthConfig.voltageCompSaturation = 12;

    TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
    driveConfig.continuousCurrentLimit = 40;
    driveConfig.peakCurrentDuration = 45;
    driveConfig.peakCurrentLimit = 40;
    driveConfig.slot0.kP = 0.05;
    driveConfig.slot0.kI = 0.0005;
    driveConfig.slot0.kD = 0.0;
    driveConfig.slot0.kF = 0.032;
    driveConfig.slot0.integralZone = 1000;
    driveConfig.slot0.maxIntegralAccumulator = 150_000;
    driveConfig.slot0.allowableClosedloopError = 0;
    driveConfig.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
    driveConfig.velocityMeasurementWindow = 64;
    driveConfig.voltageCompSaturation = 12;

    TelemetryService telemetryService = RobotContainer.TELEMETRY;
    telemetryService.stop();

    Wheel[] wheels = new Wheel[4];

    for (int i = 0; i < 4; i++) {
      TalonSRX azimuthTalon = new TalonSRX(i);
      azimuthTalon.configAllSettings(azimuthConfig);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      TalonSRX driveTalon = new TalonSRX(i + 10);
      driveTalon.configAllSettings(driveConfig);
      driveTalon.setNeutralMode(NeutralMode.Brake);
      driveTalon.enableCurrentLimit(true);
      driveTalon.enableVoltageCompensation(true);

      telemetryService.register(new TalonItem(azimuthTalon, "Azimuth " + i));
      telemetryService.register(new TalonItem(driveTalon, "Drive " + (i + 10)));

      Wheel wheel = new Wheel(azimuthTalon, driveTalon, MAX_VELOCITY);
      wheels[i] = wheel;
    }
    telemetryService.start();

    return wheels;
  }

  private SwerveDrive configSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  public void setDriveMode(DriveMode mode) {
    swerve.setDriveMode(mode);
  }

  // Pathfinder Stuff
  public void calculateTrajctory() {
    Waypoint[] points = Constants.AUTO_PATH;
    Trajectory.Config trajectoryConfig =
        new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_HIGH,
            0.05,
            MAX_VELOCITY,
            MAX_ACCELERATION,
            MAX_JERK);
    trajectoryGenerated = Pathfinder.generate(points, trajectoryConfig);
  }

  public void startPath() {
    segments = 0;
    // follower.setTrajectory(trajectoryGenerated);
    talonPosition = Math.abs(swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition());
    // follower.configureEncoder(talonPosition, TICKS_PER_REV, WHEEL_DIAMETER);
    // follower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY, 0);
    swerve.setDriveMode(DriveMode.CLOSED_LOOP);
  }

  public void updatePathOutput() {
    if (segments < trajectoryGenerated.length()) {
      double desiredDistance = trajectoryGenerated.get(segments).position;
      double currentDistance =
          Math.abs(
              swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition() / TICKS_PER_METER);
      double error = desiredDistance - currentDistance;
      double rawOutput = kP_PATH * error + kV_PATH * trajectoryGenerated.get(segments).velocity;
      // double rawOutput =
      // follower.calculate(Math.abs(swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition() - talonPosition)); // Meters per second
      double output = (rawOutput * TICKS_PER_METER) / (MAX_VELOCITY * 10); // Ticks per 100ms
      double heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(trajectoryGenerated.get(segments).heading));
      double forward = Math.cos(Pathfinder.d2r(heading)) * output,
          strafe = Math.sin(Pathfinder.d2r(heading)) * output;
      segments++;

      drive(forward, strafe, 0.0);
    }
    else {
      drive(0,0,0);
    }
  }

  public boolean isPathDone() {
   return segments >= trajectoryGenerated.length();
  }
}
