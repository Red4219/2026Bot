package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisAimCommand extends Command {

    private static DriveSubsystem driveSubsystem;

    public ChassisAimCommand() {
         driveSubsystem = RobotContainer.driveSubsystem;

         addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        //System.out.println("ChassisAimCommand::execute() called");
        driveSubsystem.ChassisAimAtTarget();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void AimAtTarget() {
		if(Constants.ShooterConstants.enabled) {

            Pose2d pose = RobotContainer.driveSubsystem.getPoseEstimatorPose2d();

			double temp = RobotContainer.shooterSubsystem.findAngleToTarget(
                RobotContainer.driveSubsystem.getPoseEstimatorPose2d()
            );

			double diff = (temp - pose.getRotation().getDegrees());

			System.out.println(
				"DriveSubsystem::ChassisAimAtTarget() - temp: " + temp 
				+ " angle: " + pose.getRotation().getDegrees()
				+ " diff: " + diff
			);


			/*if(diff < 0.5) {
				drive(0.0, 0.0, -0.2);
			} else if(diff > 0.5) {
				drive(0.0, 0.0, 0.2);
			}*/

			// Rotate to the target
			//drive(0.0, 0.0, 0.05);
			//System.out.println("DriveSubsystem::ChassisAimAtTarget() angle is: " + RobotContainer.shooterSubsystem.getAngleToTarget());
		}
	}

    public double findAngleToTarget( Pose2d currentPose) {
        
        double temp = Math.atan2(
            0.0 - currentPose.getY(),
            0.0 - currentPose.getX()
        );

        temp = Math.toDegrees(temp);

        return temp;
    }
    
}
