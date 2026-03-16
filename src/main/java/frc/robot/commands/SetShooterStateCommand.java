package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class SetShooterStateCommand extends Command {

    private static ShooterSubsystem shooterSubsystem;
    private static IntakeSubsystem intakeSubsystem;

    private ShooterState shooterState = ShooterState.Stopped;

    public SetShooterStateCommand(ShooterState shooterState) {
        shooterSubsystem = RobotContainer.shooterSubsystem;
        intakeSubsystem = RobotContainer.intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);

        this.shooterState = shooterState;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        System.out.println("set shooter command ran " + this.shooterState);
        switch(this.shooterState) {
            case Shooting:
                shooterSubsystem.shooterState = ShooterState.Shooting;
                intakeSubsystem.intakeState = IntakeState.Intake;
                break;
            case Stopped:
                shooterSubsystem.shooterState = ShooterState.Stopped;
                break;
            case Tracking:
                shooterSubsystem.shooterState = ShooterState.Tracking;
                intakeSubsystem.intakeState = IntakeState.Stop;
                break;
            case Eject:
                shooterSubsystem.shooterState = ShooterState.Eject;
                // intakeSubsystem.intakeState = IntakeState.Eject;
            case ReverseIndexer:
                shooterSubsystem.shooterState = ShooterState.ReverseIndexer;
                break;
            default:
                shooterSubsystem.shooterState = ShooterState.Stopped;
                break;

        }
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.shooterState.equals(this.shooterState);
    }
}
