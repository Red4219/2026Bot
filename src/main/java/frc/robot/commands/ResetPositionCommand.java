package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class ResetPositionCommand extends Command {
    DriveSubsystem driveSubsystem;

    public ResetPositionCommand() {
        this.driveSubsystem = RobotContainer.driveSubsystem;
      
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.zeroHeading();
        driveSubsystem.resetOdometry(new Pose2d());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
