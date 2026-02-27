package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class SetShooterStateCommand extends Command {

    private static ShooterSubsystem shooterSubsystem;
    private ShooterState shooterState = ShooterState.Stopped;

    public SetShooterStateCommand(ShooterState shooterState) {
        shooterSubsystem = RobotContainer.shooterSubsystem;
        addRequirements(shooterSubsystem);
        this.shooterState = shooterState;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        switch(shooterState) {
            case Shooting:
                shooterSubsystem.shooterState = ShooterState.Shooting;
                break;
            case Stopped:
                shooterSubsystem.shooterState = ShooterState.Stopped;
                break;
            case Tracking:
                shooterSubsystem.shooterState = ShooterState.Tracking;
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
        return true;
    }
}
