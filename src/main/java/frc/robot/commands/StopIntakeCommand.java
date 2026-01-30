package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class StopIntakeCommand extends Command {

    private static IntakeSubsystem _intakeSubsystem;

    public StopIntakeCommand() {
        _intakeSubsystem = RobotContainer.intakeSubsystem;
        addRequirements(_intakeSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        // Tell the intake subsystem that we need to eject
        _intakeSubsystem.intakeState = IntakeState.Stop;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
