package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;

public class ClimbCommand extends Command {

    private static ClimberSubsystem climberSubsystem;
    private ClimbState climbState = ClimbState.Stored;

    public ClimbCommand(ClimbState climbState) {
        climberSubsystem = RobotContainer.climberSubsystem;
        addRequirements(climberSubsystem);

        this.climbState = climbState;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        climberSubsystem.climbState = climbState;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;
    }
    
}
