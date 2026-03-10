package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class IntakeCommand extends Command {

    private static IntakeSubsystem _intakeSubsystem;
    private IntakeState intakeState = IntakeState.Stop;
    private static ShooterSubsystem _shooterSubsystem;

    public IntakeCommand(IntakeState intakeState) {
        _intakeSubsystem = RobotContainer.intakeSubsystem;
        _shooterSubsystem = RobotContainer.shooterSubsystem;
        addRequirements(_intakeSubsystem);
        this.intakeState = intakeState;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        System.out.println("ran intake command " + this.intakeState);
        switch(this.intakeState) {
            case Eject:
                _shooterSubsystem.shooterState = ShooterState.Eject;
                _intakeSubsystem.intakeState = IntakeState.Eject;
                break;
            case Intake:
                _intakeSubsystem.intakeState = IntakeState.Intake;
                break;
            case Stop:
                _intakeSubsystem.intakeState = IntakeState.Stop;
                break;
            case CollapseIntake:
                _intakeSubsystem.intakeState = IntakeState.CollapseIntake;
                break;
            default:
                break;
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return _intakeSubsystem.intakeState.equals(this.intakeState);
    }
}
