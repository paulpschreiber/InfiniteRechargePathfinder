package frc.robot.commands;

import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class PathCommand extends CommandBase {

    private DriveSubsystem driveSubsystem = RobotContainer.DRIVE;

    public PathCommand() {
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.startPath();
    }

    @Override
    public void execute() {
        driveSubsystem.updatePathOutput();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.isPathDone();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0);
        driveSubsystem.setDriveMode(DriveMode.OPEN_LOOP);
    }
}