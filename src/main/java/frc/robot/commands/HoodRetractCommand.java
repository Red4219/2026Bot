package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class HoodRetractCommand extends Command {

    private static ShooterSubsystem shooterSubsystem;
    private static IntakeSubsystem intakeSubsystem;

    private ShooterState shooterState = ShooterState.Stopped;

    public HoodRetractCommand() {
        shooterSubsystem = RobotContainer.shooterSubsystem;
        // intakeSubsystem = RobotContainer.intakeSubsystem;

        addRequirements(shooterSubsystem);

        // this.shooterState = shooterState;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // System.out.println("set shooter command ran " + this.shooterState);
        // shooterSubsystem.oldState = shooterSubsystem.shooterState;
        shooterSubsystem.shooterState = ShooterState.Stopped;

        
        
    }

    @Override
    public void end(boolean interrupted) {
        if (!shooterSubsystem.isTrenchSafe){
            new HoodRetractCommand();
        }
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.isTrenchSafe;
    }
}
