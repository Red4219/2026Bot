package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class ShootCommand extends Command{

    private static ShooterSubsystem shooterSubsystem;
    private boolean shoot = false;

    public ShootCommand(boolean shoot) {
        shooterSubsystem = RobotContainer.shooterSubsystem;
        addRequirements(shooterSubsystem);
        this.shoot = shoot;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        if(shoot) { 
            shooterSubsystem.shooterState = ShooterSubsystem.ShooterState.Shooting;
        } else {
            shooterSubsystem.shooterState = ShooterSubsystem.ShooterState.Tracking;
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
