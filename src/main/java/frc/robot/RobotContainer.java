// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ChassisAimCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ToggleTurretManualCommand;
import frc.robot.commands.ResetPositionCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.EjectCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public SendableChooser<Command> getAutoChooser() {
		return autoChooser;
	}

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotContainer.driveSubsystem.CreateAutoBuilder();
    
    // Configure the trigger bindings
    configureBindings();
    new ResetPositionCommand();

    // Register the auto commands
    registerAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser("Auto 1");

    SmartDashboard.putData("Auto", autoChooser);

    // Add the chooser to the Shuffleboard to select which Auo to run
		Shuffleboard.getTab("Autonomous").add("Auto", autoChooser)
			.withWidget(BuiltInWidgets.kComboBoxChooser);

    SmartDashboard.putData("Field", field);
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
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //autoChooser = AutoBuilder.buildAutoChooser("Auto 1");
    //autoChooser = AutoBuilder.buildAutoChooser();

    //driverController.button(1).whileTrue(new ChassisAimCommand());

    // SmartDashboard.putData("Auto", autoChooser);

    // // Add the chooser to the Shuffleboard to select which Auo to run
		// Shuffleboard.getTab("Autonomous").add("Auto", autoChooser)
		// 	.withWidget(BuiltInWidgets.kComboBoxChooser);
	


    if(RobotBase.isReal()) {

      // Driver to reset field oriented drive
			driverController.button(8).whileTrue(new ResetPositionCommand());

      driverController.rightTrigger().onTrue(new ShootCommand(true));
      driverController.rightTrigger().onFalse(new ShootCommand(false));
    
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

      operatorController.button(1).whileTrue(new ToggleTurretManualCommand());
      driverController.button(2).onTrue(new ShootCommand(true));
      driverController.button(2).onFalse(new ShootCommand(false));
      driverController.button(3).onTrue(new IntakeCommand(true));
      driverController.button(3).onFalse(new IntakeCommand(false));
      driverController.button(4).onTrue(new ClimbCommand(ClimbState.Up));
      driverController.button(4).onFalse(new ClimbCommand(ClimbState.Stored));

      driveSubsystem.setDefaultCommand(

					new RunCommand(() -> driveSubsystem.drive(
							JoystickUtils.processJoystickInput(-driverController.getLeftY()),
							JoystickUtils.processJoystickInput(-driverController.getLeftX()),
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

  private void registerAutoCommands() {
    NamedCommands.registerCommand("ClimbUp", new ClimbCommand(ClimbState.Up));
    NamedCommands.registerCommand("ClimbDown", new ClimbCommand(ClimbState.Down));
    NamedCommands.registerCommand("Intake", new IntakeCommand(true));
    NamedCommands.registerCommand("StopIntake", new IntakeCommand(false));
    //NamedCommands.registerCommand("StopIntake", new StopIntakeCommand());
    NamedCommands.registerCommand("Eject", new EjectCommand());
    NamedCommands.registerCommand("ShootStart", new ShootCommand(true));
    NamedCommands.registerCommand("ShootStop", new ShootCommand(false));
  }

  public void simulationInit() {
    driveSubsystem.simulationInit();
    shooterSubsystem.simulationInit();
    intakeSubsystem.simulationInit();
    climberSubsystem.simulationInit();
  }
}
