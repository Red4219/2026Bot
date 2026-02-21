package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class IntakeCommand extends Command {

    private static IntakeSubsystem _intakeSubsystem;
    private boolean intake = false;

    public IntakeCommand(boolean intake) {
        _intakeSubsystem = RobotContainer.intakeSubsystem;
        addRequirements(_intakeSubsystem);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        //System.out.println("IntakeCommand::initialize() called");

        // Tell the intake subsystem that we need to intake
        //_intakeSubsystem.intakeState = IntakeState.Intake;
    }

    @Override
    public void execute() {
        if(intake) {
            _intakeSubsystem.intakeState = IntakeState.Intake;
        } else {
            _intakeSubsystem.intakeState = IntakeState.Stop;
        }
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("IntakeCommand::end() called, interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
