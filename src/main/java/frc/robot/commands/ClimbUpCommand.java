package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;

public class ClimbUpCommand extends Command {

    private static ClimberSubsystem _climberSubsystem;

    public ClimbUpCommand() {
         _climberSubsystem = RobotContainer.climberSubsystem;

         addRequirements(_climberSubsystem);
    }

    @Override
    public void initialize() {
        // Tell the Climber to go up
        _climberSubsystem.climbState = ClimbState.Up;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {

        // Only return true when we have successfully gone up
        /*if(_climberSubsystem.atTargetPosition()){
            return true;
        }*/

        // Target position has not been met so return false
        //return false;
        return true;
    }
    
}
