package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;

public class NoClimbCommand extends Command {

    private static DriveSubsystem _driveSubsystem;
    private static ClimberSubsystem _climberSubsystem;
    private boolean _finished = false;

    public NoClimbCommand() {
         _driveSubsystem = RobotContainer.driveSubsystem;
         _climberSubsystem = RobotContainer.climberSubsystem;

         addRequirements(_driveSubsystem, _climberSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        // Tell the climber subsystem that we need to no climb
        _climberSubsystem.climbState = ClimbState.NoClimb;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        // Only return true when we have successfully climbed
        if(_climberSubsystem.atTargetPosition()){
            return true;
        }

        // Target position has not been met so return false
        return false;
    }
    
}
