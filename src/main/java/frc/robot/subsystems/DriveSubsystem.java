package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;
import org.strykeforce.thirdcoast.telemetry.item.Measurable;
import org.strykeforce.thirdcoast.telemetry.item.Measure;
import org.strykeforce.thirdcoast.telemetry.item.TalonItem;

public class DriveSubsystem extends SubsystemBase implements Measurable {

  private static TelemetryService telemetryService;

  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;
  private static final double MAX_VELOCITY = 10000;
  private static final double MAX_ACCELERATION = 2.0;
  private static final double MAX_JERK = 60.0;
  private static final int TICKS_PER_REV = 9011;
  private static final double WHEEL_DIAMETER = 0.0635; // In meters
  private static final double TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
  private static final double kP_PATH = 0.7;
  private static final double MAX_VELOCITY_MPS = (MAX_VELOCITY * 10) / TICKS_PER_METER;
  private static final double kV_PATH = 1 / MAX_VELOCITY_MPS;

  private final SwerveDrive swerve = configSwerve();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  private Trajectory trajectoryGenerated;
  private int talonPosition;
  private double estimatedDistanceTraveled;
  private Translation2d lastPosition;
  private double currentDistance;
  private double desiredDistance;
  private double error;

  // Odometry stuff

  private Translation2d[] wheelsMeters = {new Translation2d(Units.inchesToMeters(ROBOT_LENGTH / 2.0), -Units.inchesToMeters(ROBOT_WIDTH / 2.0)), new Translation2d(Units.inchesToMeters(ROBOT_LENGTH / 2.0), Units.inchesToMeters(ROBOT_WIDTH / 2.0)), new Translation2d(-Units.inchesToMeters(ROBOT_LENGTH / 2.0), -Units.inchesToMeters(ROBOT_WIDTH / 2.0)), new Translation2d(-Units.inchesToMeters(ROBOT_LENGTH / 2.0), Units.inchesToMeters(ROBOT_WIDTH / 2.0))};
  private SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(new SwerveDriveKinematics(wheelsMeters), gyroAngle);

  public DriveSubsystem() {
    swerve.setFieldOriented(true);
    zeroAzimuth();
    telemetryService = RobotContainer.TELEMETRY;
    telemetryService.stop();
    telemetryService.register(this);
    telemetryService.start();
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

  public void zeroAzimuth() {
    swerve.zeroAzimuthEncoders();
  }

  // Pathfinder Stuff
  public void calculateTrajctory() {
    List<Translation2d> path = Constants.INTERNAL_POINTS;
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY_MPS, MAX_ACCELERATION);

    trajectoryGenerated =
        TrajectoryGenerator.generateTrajectory(
            Constants.START_PATH, path, Constants.END_PATH, trajectoryConfig);
  }

  public void startPath() {
    // follower.setTrajectory(trajectoryGenerated);
    talonPosition = Math.abs(swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition());
    lastPosition = Constants.START_PATH.getTranslation();
    estimatedDistanceTraveled = 0;
    desiredDistance = 0;
    // follower.configureEncoder(talonPosition, TICKS_PER_REV, WHEEL_DIAMETER);
    // follower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY, 0);
    swerve.setDriveMode(DriveMode.CLOSED_LOOP);
  }

  public void updatePathOutput(double timeSeconds) {
    Trajectory.State currentState = trajectoryGenerated.sample(timeSeconds);
    estimatedDistanceTraveled += currentState.poseMeters.getTranslation().getDistance(lastPosition);
    desiredDistance = estimatedDistanceTraveled;
    currentDistance =
        Math.abs(swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition() - talonPosition)
            / TICKS_PER_METER;
    System.out.println("Desired dist: " + desiredDistance + " Current dist: " + currentDistance);
    error = desiredDistance - currentDistance;
    double rawOutput = kP_PATH * error + kV_PATH * currentState.velocityMetersPerSecond;
    // double rawOutput =
    // follower.calculate(Math.abs(swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition()
    // - talonPosition)); // Meters per second
    double output = (rawOutput * TICKS_PER_METER) / (MAX_VELOCITY * 10); // Ticks per 100ms
    double heading = currentState.poseMeters.getRotation().getRadians();
    double forward = Math.cos(heading) * output, strafe = Math.sin(heading) * output;
    drive(forward, strafe, 0.0);
    lastPosition = currentState.poseMeters.getTranslation();
  }

  public boolean isPathDone(double timePassedSeconds) {
    return timePassedSeconds >= trajectoryGenerated.getTotalTimeSeconds();
  }

  public void startPathOdometry() {
    talonPosition = Math.abs(swerve.getWheels()[0].getDriveTalon().getSelectedSensorPosition());
  }

  public void updatePathOdometry(double timeSeconds) {

  }

  // Telemetry stuff

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("DesiredDistance", "DesiredDistance"),
        new Measure("CurrentDistance", "CurrentDistance"),
        new Measure("DistanceError", "DistanceError"));
  }

  @Override
  public DoubleSupplier measurementFor(Measure arg0) {
    switch (arg0.getName()) {
      case "DesiredDistance":
        return () -> desiredDistance;
      case "CurrentDistance":
        return () -> currentDistance;
      case "DistanceError":
        return () -> error;
      default:
        return () -> -2767.0;
    }
  }

  @Override
  public int compareTo(Measurable arg0) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public String getType() {
    // TODO Auto-generated method stub
    return "AutoPathing";
  }

  @Override
  public String getDescription() {
    // TODO Auto-generated method stub
    return "AutoPathing";
  }

  @Override
  public int getDeviceId() {
    // TODO Auto-generated method stub
    return 0;
  }
}
