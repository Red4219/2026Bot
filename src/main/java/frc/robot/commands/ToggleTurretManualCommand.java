package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleTurretManualCommand extends Command {

    private ShooterSubsystem shooterSubsystem;

    public ToggleTurretManualCommand() {
        this.shooterSubsystem = RobotContainer.shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        // Flip it to the opposite to what it was
        shooterSubsystem.setManualControl(!shooterSubsystem.getManualControl());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
