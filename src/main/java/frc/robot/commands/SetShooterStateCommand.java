package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class SetShooterStateCommand extends Command {

    private static ShooterSubsystem shooterSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static DriveSubsystem driveSubsystem;

    private ShooterState shooterState = ShooterState.Stopped;

    public SetShooterStateCommand(ShooterState shooterState) {
        shooterSubsystem = RobotContainer.shooterSubsystem;
        intakeSubsystem = RobotContainer.intakeSubsystem;
        driveSubsystem = RobotContainer.driveSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem, driveSubsystem);

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
                driveSubsystem.driveState = DriveState.Slow;
                break;
            case Stopped:
                shooterSubsystem.shooterState = ShooterState.Stopped;
                driveSubsystem.driveState = DriveState.Normal;
                break;
            case Tracking:
                shooterSubsystem.shooterState = ShooterState.Tracking;
                intakeSubsystem.intakeState = IntakeState.Stop;
                driveSubsystem.driveState = DriveState.Normal;
                break;
            case Eject:
                shooterSubsystem.shooterState = ShooterState.Eject;
                driveSubsystem.driveState = DriveState.Normal;
            case ReverseIndexer:
                shooterSubsystem.shooterState = ShooterState.ReverseIndexer;
                driveSubsystem.driveState = DriveState.Normal;
                break;
            default:
                shooterSubsystem.shooterState = ShooterState.Stopped;
                driveSubsystem.driveState = DriveState.Normal;
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
