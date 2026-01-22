// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ResetPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final Field2d field = new Field2d();
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public SendableChooser<Command> getAutoChooser() {
		return autoChooser;
	}

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotContainer.driveSubsystem.CreateAutoBuilder();
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //autoChooser = AutoBuilder.buildAutoChooser("Auto 1");
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto", autoChooser);

    // Add the chooser to the Shuffleboard to select which Auo to run
		Shuffleboard.getTab("Autonomous").add("Auto", autoChooser)
			.withWidget(BuiltInWidgets.kComboBoxChooser);
	


    if(RobotBase.isReal()) {

      // Driver to reset field oriented drive
			driverController.button(8).whileTrue(new ResetPositionCommand());
    
      driveSubsystem.setDefaultCommand(
			  new RunCommand(() -> driveSubsystem.drive(
				  JoystickUtils.processJoystickInput(driverController.getLeftY()),
				  JoystickUtils.processJoystickInput(driverController.getLeftX()),
				  JoystickUtils.processJoystickInput(-driverController.getRightX())
			  ),
			  driveSubsystem
			  )
		  );
    } else {
      driveSubsystem.setDefaultCommand(

					new RunCommand(() -> driveSubsystem.drive(
							JoystickUtils.processJoystickInput(driverController.getLeftY()),
							JoystickUtils.processJoystickInput(driverController.getLeftX()),
							JoystickUtils.processJoystickInput(-driverController.getRawAxis(2))
						),
						driveSubsystem
					)
				);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }

  public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
