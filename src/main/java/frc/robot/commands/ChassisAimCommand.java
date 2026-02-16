package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;

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
    
}
