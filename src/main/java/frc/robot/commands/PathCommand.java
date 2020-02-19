package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;

public class PathCommand extends CommandBase {

  private DriveSubsystem driveSubsystem = RobotContainer.DRIVE;
  private double startTimeSeconds;

  public PathCommand() {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.startPath();
    startTimeSeconds = System.currentTimeMillis() / 100;
  }

  @Override
  public void execute() {
    double currentTimeSeconds = System.currentTimeMillis() / 100;
    driveSubsystem.updatePathOutput(currentTimeSeconds - startTimeSeconds);
  }

  @Override
  public boolean isFinished() {
    return driveSubsystem.isPathDone(startTimeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
    driveSubsystem.setDriveMode(DriveMode.OPEN_LOOP);
  }
}
