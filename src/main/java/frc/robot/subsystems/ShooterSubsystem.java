package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.node.POJONode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterState {
        Stopped,
        Shooting,
        Tracking
    }

    public enum ShooterTarget {
        BlueTower,
        RedTower,
        BlueHumanElement,
        RedHumanElement,
        BlueWall,
        RedWall
    }

    //public Map<Integer, Translation2d> targets = new HashMap<Integer, Translation2d>();

    public Translation2d[] targets = {
        new Translation2d(4.5,4.0), // Blue tower
        new Translation2d(12.0, 4.0), // Red tower
        new Translation2d(0.0, 1.0), // Blue Human Element
        new Translation2d(16.6, 7.0), // Red Human Element
        new Translation2d(0.0, 7.0), // Blue Wall
        new Translation2d(16.5, 1.0) // Red Wall 
    };

    private boolean manualControl = false;

    private String stateText = "Stopped";

    // Fly wheel motors
    //private SparkFlex flywheelMotor = null;
    private TalonFX flywheelMotor1 = null;
	private TalonFXSimState flywheel1SimState = null;
    private DCMotorSim flywheelMotor1SimModel = null;
    private TalonFXConfiguration flywheelMotor1Config = null;

    private TalonFX flywheelMotor2 = null;
	private TalonFXSimState flywheel2SimState = null;
    private DCMotorSim flywheelMotor2SimModel = null;
    private TalonFXConfiguration flywheelMotor2Config = null;


    // Turret motor, this turns the turret around
    private SparkMax turretMotor = null;
    private SparkMaxSim turretMotorSim = null;
    private SparkRelativeEncoder relativeEncoderTurret = null;
    private SparkRelativeEncoderSim relativeEncoderTurretSim = null;
    private double motorPositionTurret = 0.0;

    // This is the indexer motor, it spins the indexer around to feed the kicker/turret
    private SparkFlex indexerMotor = null;
    private SparkFlexSim indexerMotorSim = null;

    // These are the kicker motors, that pull the ball up from the indexer and pushed into the turret
    private SparkFlex kickMotor1 = null;
    private SparkFlexSim kickMotor1Sim = null;
    private SparkFlex kickMotor2 = null;
    private SparkFlexSim kickMotor2Sim = null;


    //private SparkMax hoodMotor = null;
    private double turretTargetAngle = 0.0;

    // These are the hood actuators to lift/lower the hood
    private Servo hoodActuator1 = null;
    private Servo hoodActuator2 = null;

    private double targetFlyWheelSpeed = 0.0;
    private double actualFlyWheelSpeed = 0.0;
    private double targetHoodValue = 0.0;
    private double actualHoodValue = 0.0;
    private double targetTurretPosition = 0.0;
    private double actualTurretPosition = 0.0;
    private double turretAngle = 0.0;

    private double distanceFromTarget = 0.0;

    public ShooterState shooterState = ShooterState.Stopped;

    private NetworkTableInstance inst = null;
	private NetworkTable table = null;

    private DoubleTopic topicFlywheelVelocity = null;
	private DoublePublisher pubFlywheelVelocity = null;

    private DoubleTopic topicHoodPosition = null;
	private DoublePublisher pubHoodPosition = null;

    private DoubleTopic topicTurretPosition = null;
	private DoublePublisher pubTurretPosition = null;

    private DoubleTopic topicDistanceFromTarget = null;
	private DoublePublisher pubDistanceFromTarget = null;

    private StringTopic topicStateString = null;
	private StringPublisher pubStateString = null;

    private StringTopic topicTargetString = null;
	private StringPublisher pubTargetString = null;

    private BooleanTopic topicManualControl = null;
	private BooleanPublisher pubManualControl = null;

    private Limelight limelight = null;

    private double angleToTarget = 0.0;

    private Translation2d target = targets[1]; // default point to the red tower

    //private AprilTagFieldLayout aprilTagFieldLayout = null;

    private ShooterTarget shootTarget = ShooterTarget.RedTower;
    private String shootTargetText = "Red Tower";

    private Pose2d currentPosition = new Pose2d();
    private FieldObject2d field;

    public ShooterSubsystem() {

        if (ShooterConstants.enabled) {

            /*try {
                aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
            } catch (Exception e) {
                System.out.println("ShooterSubsystem::constructor - " + e.getMessage());
            }*/

            // Flywheel Motor
            //flywheelMotor = new SparkFlex(ShooterConstants.flywheelMotor1Id, MotorType.kBrushless);
            

            // Setup the config for the motors
            // SparkFlexConfig flyWheelConfig = new SparkFlexConfig();
            // flyWheelConfig
            //         .idleMode(IdleMode.kCoast)
            //         .inverted(ShooterConstants.invertFlywheelMotor1).closedLoop
            //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //         .pid(
            //                 ShooterConstants.flywheel1P,
            //                 ShooterConstants.flywheel1I,
            //                 ShooterConstants.flywheel1D);

            // flyWheelConfig.signals.primaryEncoderPositionPeriodMs(5);

            // Hood Motor
            //hoodMotor = new SparkMax(ShooterConstants.hoodMotorId, MotorType.kBrushless);
            hoodActuator1 = new Servo(Constants.ShooterConstants.hoodActuator1Port);
            hoodActuator2 = new Servo(Constants.ShooterConstants.hoodActuator2Port);

            // Setup the config for the motors
            // SparkFlexConfig hoodConfig = new SparkFlexConfig();
            // hoodConfig
            //         .idleMode(IdleMode.kCoast)
            //         .inverted(ShooterConstants.invertHoodMotor).closedLoop
            //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //         .pid(
            //                 ShooterConstants.hoodP,
            //                 ShooterConstants.hoodI,
            //                 ShooterConstants.hoodD);

            // hoodConfig.signals.primaryEncoderPositionPeriodMs(5);

            // Indexer Motor
            indexerMotor = new SparkFlex(ShooterConstants.indexerMotorId, MotorType.kBrushless);
            SparkFlexConfig indexConfig = new SparkFlexConfig();
            indexConfig
                    .smartCurrentLimit(ShooterConstants.indexerMotorCurrentLimit)
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertIndexerMotor).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            indexerMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Kick Motor 1
            kickMotor1 = new SparkFlex(ShooterConstants.kickMotor1Id, MotorType.kBrushless);
            SparkFlexConfig kick1Config = new SparkFlexConfig();
            kick1Config
                    .smartCurrentLimit(ShooterConstants.kickMotorCurrentLimit)
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertFlywheelMotor1).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            kickMotor1.configure(kick1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Kick Motor 2
            kickMotor2 = new SparkFlex(ShooterConstants.kickMotor2Id, MotorType.kBrushless);
            SparkFlexConfig kick2Config = new SparkFlexConfig();
            kick2Config
                    .smartCurrentLimit(ShooterConstants.kickMotorCurrentLimit)
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertFlywheelMotor2).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            kickMotor2.configure(kick2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Turret Motor
            turretMotor = new SparkMax(ShooterConstants.turretMotorId, MotorType.kBrushless);
            SparkMaxConfig turretConfig = new SparkMaxConfig();
            
            turretConfig
                .idleMode(IdleMode.kBrake)
                .inverted(ShooterConstants.invertTurretMotor)
                .smartCurrentLimit(ShooterConstants.turretMotorCurrentLimit)
                .closedLoop
                //.outputRange(-10.0, 10.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                        ShooterConstants.turretP,
                        ShooterConstants.turretI,
                        ShooterConstants.turretD
                    );

            // turretConfig.closedLoop.apply(new MAXMotionConfig()
            //     .cruiseVelocity(ShooterConstants.turretMotorMaxMotionCruiseVelocity)
            //     .maxAcceleration(ShooterConstants.turretMotorMaxMotionMaxAcceleration)
            //     .allowedProfileError(ShooterConstants.turretMotorMaxMotionAllowedProfileError)
            //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            //     // .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
            //     // .cruiseVelocity(actualFlyWheelSpeed, ClosedLoopSlot.kSlot0)
            //     // .maxAcceleration(actualFlyWheelSpeed, ClosedLoopSlot.kSlot0)
            //     // .allowedProfileError(ShooterConstants.turretMotorMaxMotionAllowedProfileError, ClosedLoopSlot.kSlot0)
            // );
            
            // turretConfig.closedLoop.apply(new FeedForwardConfig()
            //     .kS(0.0, ClosedLoopSlot.kSlot0)
            //     .kV(0.0, ClosedLoopSlot.kSlot0)
            //     //.kV(12.0 / 5767, ClosedLoopSlot.kSlot0)
            //     //.kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
            //     .kA(0.0, ClosedLoopSlot.kSlot0)
            // );

            turretConfig.signals.primaryEncoderPositionPeriodMs(5);
            turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            //turretConfig.encoder.positionConversionFactor(30.0);
            relativeEncoderTurret = (SparkRelativeEncoder) turretMotor.getEncoder();
            motorPositionTurret = relativeEncoderTurret.getPosition();

            // Setup the network tables info
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("ShooterSubsystem");

            topicFlywheelVelocity = table.getDoubleTopic("Flywheel Velocity");
            pubFlywheelVelocity = topicFlywheelVelocity.publish();
            pubFlywheelVelocity.set(actualFlyWheelSpeed);

            topicHoodPosition = table.getDoubleTopic("Hood Position");
            pubHoodPosition = topicHoodPosition.publish();
            pubHoodPosition.set(actualHoodValue);

            topicTurretPosition = table.getDoubleTopic("Turret Position");
            pubTurretPosition = topicTurretPosition.publish();
            pubTurretPosition.set(actualTurretPosition);

            topicDistanceFromTarget = table.getDoubleTopic("Distance from Target");
            pubDistanceFromTarget = topicDistanceFromTarget.publish();
            pubDistanceFromTarget.set(distanceFromTarget);

            topicStateString = table.getStringTopic("StateString");
            pubStateString = topicStateString.publish();
            pubStateString.set(stateText);

            topicTargetString = table.getStringTopic("TargetString");
            pubTargetString = topicTargetString.publish();
            pubTargetString.set(shootTargetText);

            topicManualControl = table.getBooleanTopic("ManualControl");
            pubManualControl = topicManualControl.publish();
            pubManualControl.set(manualControl);

            limelight = new Limelight("limelight");

            //target = aprilTagFieldLayout.getTagPose(10).get().getTranslation().toTranslation2d();
            // Look at the center of the red target
            //target = new Translation2d(12.0, 4.0);

            flywheelMotor1 = new TalonFX(ShooterConstants.flywheelMotor1Id, Constants.kCanivoreCANBusName);
            flywheelMotor2 = new TalonFX(ShooterConstants.flywheelMotor2Id, Constants.kCanivoreCANBusName);

            // Flywheel Motor 1
            flywheelMotor1Config = new TalonFXConfiguration();
		    flywheelMotor1Config
		        .withMotorOutput(
			        new MotorOutputConfigs()
				        .withInverted(InvertedValue.Clockwise_Positive)
				        .withNeutralMode(NeutralModeValue.Coast)
		        );
		    flywheelMotor1Config.Audio.AllowMusicDurDisable = true;
		
		    flywheelMotor1.getConfigurator().apply(flywheelMotor1Config);

		    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
		    currentConfig.StatorCurrentLimitEnable = true;
		    currentConfig.StatorCurrentLimit = Constants.ShooterConstants.flywheel1CurrentLimit;
		    flywheelMotor1.getConfigurator().apply(currentConfig);

		    var slot0Configs = new Slot0Configs();
		    slot0Configs.kS = 0.1;
		    slot0Configs.kV = 0.12;
		    slot0Configs.kP = Constants.ShooterConstants.flywheel1P; // An error of 1 rps results in 0.11 V output
		    slot0Configs.kI = Constants.ShooterConstants.flywheel1I; // no output for integrated error
		    slot0Configs.kD = Constants.ShooterConstants.flywheel1D; // no output for error derivative
		    flywheelMotor1.getConfigurator().apply(slot0Configs);

            // Flywheel Motor 2
            flywheelMotor2Config = new TalonFXConfiguration();
		    flywheelMotor2Config
		        .withMotorOutput(
			        new MotorOutputConfigs()
				        .withInverted(InvertedValue.Clockwise_Positive)
				        .withNeutralMode(NeutralModeValue.Coast)
		        );
		    flywheelMotor1Config.Audio.AllowMusicDurDisable = true;
		
		    flywheelMotor1.getConfigurator().apply(flywheelMotor1Config);

		    CurrentLimitsConfigs currentConfig2 = new CurrentLimitsConfigs();
		    currentConfig2.StatorCurrentLimitEnable = true;
		    currentConfig2.StatorCurrentLimit = Constants.ShooterConstants.flywheel2CurrentLimit;
		    flywheelMotor2.getConfigurator().apply(currentConfig2);
		    flywheelMotor2.getConfigurator().apply(slot0Configs);
        }
    }

    @Override
	public void periodic() {

        if (ShooterConstants.enabled) {

            // Get the position from the drive system
            currentPosition = RobotContainer.driveSubsystem.getPoseEstimatorPose2d();

            // we need a function for this to calculate the angle
            turretAngle = RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getRotation().getDegrees();

            switch (shooterState) {
                case Shooting:
                    // The turret is tracking the target, spinning the fly wheel, and we are kicking the fuel
                    // up to the turret to be shot
                    stateText = "Shooting";
                    indexerMotor.getClosedLoopController().setSetpoint(ShooterConstants.indexMotorSpeed, ControlType.kVelocity);
                    kickMotor1.getClosedLoopController().setSetpoint(ShooterConstants.kickMotor1Speed, ControlType.kVelocity);
                    kickMotor2.getClosedLoopController().setSetpoint(ShooterConstants.kickMotor2Speed, ControlType.kVelocity);
                    break;
                case Stopped:
                    // Stop everything
                    stateText = "Stopped";
                    indexerMotor.set(0.0);
                    kickMotor1.set(0.0);
                    kickMotor2.set(0.0);
                    break;
                case Tracking:
                    // The turret is tracking the target, spinning the fly wheel, but not spinning/kicking the fuel
                    // up to the turret
                    stateText = "Tracking";
                    indexerMotor.set(0.0);
                    kickMotor1.set(0.0);
                    kickMotor2.set(0.0);
                    break;
            }

            

            // Check where we are and change the shooter accordingly
            // This will decide where to shoot to fuel to
            if (DriverStation.getAlliance().isPresent()) {

                if (DriverStation.getAlliance().get() == Alliance.Red) {

                    // We are on the Red alliance 

                    if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getX() > 11.5) {
                        shootTarget = ShooterTarget.RedTower;
                        shootTargetText = "Red Tower";
                        target = targets[1];

                        actualFlyWheelSpeed = calculateFlywheelSpeed(10, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() < 4.0) {
                        shootTarget = ShooterTarget.RedWall;
                        shootTargetText = "Red Wall";
                        target = targets[5];

                        actualFlyWheelSpeed = calculateFlywheelSpeed(-1, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() > 4.0) {
                        shootTarget = ShooterTarget.RedHumanElement;
                        shootTargetText = "Red Human Element";
                        target = targets[3];

                        actualFlyWheelSpeed = calculateFlywheelSpeed(13, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    }
                } else if (DriverStation.getAlliance().get() == Alliance.Blue) {

                    // We are on the blue alliance

                    if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getX() < 4.0) {
                        shootTarget = ShooterTarget.BlueTower;
                        shootTargetText = "Blue Tower";
                        target = targets[0];

                        actualFlyWheelSpeed = calculateFlywheelSpeed(25, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() < 4.0) {
                        shootTarget = ShooterTarget.BlueHumanElement;
                        shootTargetText = "Blue Human Element";
                        target = targets[2];

                        actualFlyWheelSpeed = calculateFlywheelSpeed(30, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() > 4.0) {
                        shootTarget = ShooterTarget.BlueWall;
                        shootTargetText = "Blue Wall";
                        target = targets[4];

                        actualFlyWheelSpeed = calculateFlywheelSpeed(-1, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    }
                }

                calculateHood();
            }

            
            // This does the math to find the angle to the target using the
            // robots position on the field
            //double angle = findAngleToTarget(target, currentPosition);
            turretTargetAngle = findAngleToTarget(target, currentPosition);

            // Tell the motors where they need to be
            //flywheelMotor.getClosedLoopController().setSetpoint(targetFlyWheelSpeed, ControlType.kVelocity);
            //hoodMotor.getClosedLoopController().setSetpoint(targetHoodValue, ControlType.kPosition);
            //turretMotor.getClosedLoopController().setSetpoint(targetTurretPosition, ControlType.kPosition);
            //turretMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
            //turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, ControlType.kPosition);
            //turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, SparkBase.ControlType.kMAXMotionPositionControl);

            //if(turretTargetAngle < -175)
            //turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
            //turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, ControlType.kPosition);
            //turretMotor.getClosedLoopController().setSetpoint(10.0, SparkBase.ControlType.kMAXMotionPositionControl);

            //
            //turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
            //turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
            
            turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, ControlType.kPosition);


            // Get where the motors actually are
            //actualFlyWheelSpeed = flywheelMotor.getAbsoluteEncoder().getVelocity();
            //actualHoodValue = hoodMotor.getAbsoluteEncoder().getPosition();
            actualTurretPosition = turretMotor.getEncoder().getPosition();

            // Publish to network tables the values
            pubFlywheelVelocity.set(actualFlyWheelSpeed);
            pubHoodPosition.set(actualHoodValue);
            //pubTurretPosition.set(actualTurretPosition);
            pubTurretPosition.set(turretTargetAngle);
            pubDistanceFromTarget.set(distanceFromTarget);
            pubStateString.set(stateText);
            pubTargetString.set(shootTargetText);
            pubManualControl.set(manualControl);

            

            // Show the position on the map
            currentPosition = new Pose2d(
                currentPosition.getX(), 
                currentPosition.getY(), 
                //new Rotation2d(Math.toRadians(angle))
                //new Rotation2d(Math.toRadians(turretTargetAngle))
                new Rotation2d(Math.toRadians(turretMotor.getEncoder().getPosition() + currentPosition.getRotation().getDegrees()))
            );

            field = RobotContainer.field.getObject("My Objects");

            field.setPoses(List.of(
                currentPosition
            ));            

            if(manualControl) {
                moveLaterally(RobotContainer.operatorController.getLeftY());
            }
        }
    }

    public void simulationInit() {
        if (ShooterConstants.enabled) {
            flywheelMotor1SimModel = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60Foc(1),
                            0.001,
                            1.0),
                    DCMotor.getKrakenX60Foc(1));

            flywheel1SimState = flywheelMotor1.getSimState();
            flywheel1SimState.Orientation = ChassisReference.CounterClockwise_Positive;
            flywheel1SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

            // Motor 2
            flywheelMotor2SimModel = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60Foc(1),
                            0.001,
                            1.0),
                    DCMotor.getKrakenX60Foc(1));

            flywheel2SimState = flywheelMotor1.getSimState();
            flywheel2SimState.Orientation = ChassisReference.CounterClockwise_Positive;
            flywheel2SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

            indexerMotorSim = new SparkFlexSim(indexerMotor, DCMotor.getNeoVortex(1));
            indexerMotorSim.setBusVoltage(12.0);

            // Turret motor
            turretMotorSim = new SparkMaxSim(turretMotor, DCMotor.getNEO(1));
            turretMotorSim.setBusVoltage(12.0);
            relativeEncoderTurretSim = turretMotorSim.getRelativeEncoderSim();

            // Kick motor 1
            kickMotor1Sim = new SparkFlexSim(kickMotor1, DCMotor.getNeoVortex(1));
            kickMotor1Sim.setBusVoltage(12.0);

            // Kick motor 2
            kickMotor2Sim = new SparkFlexSim(kickMotor2, DCMotor.getNeoVortex(1));
            kickMotor2Sim.setBusVoltage(12.0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (ShooterConstants.enabled) {
            flywheel1SimState = flywheelMotor1.getSimState();

            // set the supply voltage of the TalonFX
            flywheel1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            var motorVoltage = flywheel1SimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            flywheelMotor1SimModel.setInputVoltage(motorVoltage.in(Volts));
            flywheelMotor1SimModel.update(0.020); // assume 20 ms loop time

            // Motor 2
            flywheel2SimState = flywheelMotor2.getSimState();

            // set the supply voltage of the TalonFX
            flywheel2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            motorVoltage = flywheel2SimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            flywheelMotor2SimModel.setInputVoltage(motorVoltage.in(Volts));
            flywheelMotor2SimModel.update(0.020); // assume 20 ms loop time

            // 1. Get the motor speed (voltage) from the simulation
            double motorSpeed = turretMotorSim.getAppliedOutput();

            // 2. Simulate the movement (e.g., update position based on speed)
            // In a real simulation, you would use physics models here (WPILib) 
            turretMotorSim.setVelocity(motorSpeed * 5676);
            turretMotorSim.setPosition(turretMotorSim.getPosition() + (motorSpeed * 0.1));

            // 3. Update simulation sensors
            turretMotorSim.iterate(motorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate

            switch (shooterState) {
                case Shooting:
                    stateText = "Shooting";
                    indexerMotorSim.iterate(ShooterConstants.indexMotorSpeed, 12.0, 0.2);
                    kickMotor1Sim.iterate(ShooterConstants.kickMotor1Speed, 12.0, 0.2);
                    kickMotor2Sim.iterate(ShooterConstants.kickMotor2Speed, 12.0, 0.2);
                    break;
                case Stopped:
                    stateText = "Stopped";
                    indexerMotorSim.iterate(0.0, 12.0, 0.2);
                    kickMotor1Sim.iterate(0.0, 12.0, 0.2);
                    kickMotor2Sim.iterate(0.0, 12.0, 0.2);
                    break;
                case Tracking:
                    stateText = "Tracking";
                    indexerMotorSim.setVelocity(0.0);
                    indexerMotorSim.iterate(0.0, 12.0, 0.2);
                    kickMotor1Sim.iterate(0.0, 12.0, 0.2);
                    kickMotor2Sim.iterate(0.0, 12.0, 0.2);
                    break;
            }
        }
    }

    public void setTarget(ShooterTarget target) {

    }

    private double calculateFlywheelSpeed(int targetId, Translation2d target, Pose2d currentPose) {

        if(targetId > 0 && Constants.kEnableLimelight) {
            if(limelight.lookForTarget(targetId)) {
                distanceFromTarget = limelight.getDistancToTargetFromRobot(targetId);
            } else {
                // calculate the distance based off of telemetry
                distanceFromTarget =  Math.sqrt(Math.pow(target.getX() - currentPose.getX(), 2.0) + Math.pow(target.getY() - currentPose.getY(), 2.0));

                // Publish to network tables
                pubDistanceFromTarget.set(distanceFromTarget);
            }
        } else {
            // targetId must be -1 which means we need to hit the wall
            // calculate the distance based off of telemetry
            distanceFromTarget =  Math.sqrt(Math.pow(target.getX() - currentPose.getX(), 2.0) + Math.pow(target.getY() - currentPose.getY(), 2.0));

            // Publish to network tables
            pubDistanceFromTarget.set(distanceFromTarget);
        }

        return distanceFromTarget;
        
    }

    private void calculateHood() {
        // This needs to be filled in with a function to calculate the flywheel speed

        // This needs to be replaced with a function or a lookup table
        targetHoodValue = distanceFromTarget;

        hoodActuator1.setPosition(targetHoodValue);
        hoodActuator2.setPosition(targetHoodValue);

        actualHoodValue = hoodActuator1.getPosition();
    }

    public double findAngleToTarget(Translation2d target, Pose2d currentPose) {

        double temp = 0.0;

        if (DriverStation.getAlliance().isPresent() &&  DriverStation.getAlliance().get() == Alliance.Blue) {
            temp = Math.atan2(
                target.getY() - currentPose.getY(),
                target.getX() - currentPose.getX()
            );

            temp = Math.toDegrees(temp);
        } else {
            temp = Math.atan2(
                -target.getY() - (-currentPose.getY()),
                -target.getX() - (-currentPose.getX())
            );

            temp = Math.toDegrees(temp);
            temp = (temp + 180.0) % 360.0;
        }

        //temp = Math.toDegrees(temp);

        //temp = (currentPose.getRotation().getDegrees() - temp) + currentPose.getRotation().getDegrees();

        // Find the relative angle
        temp = temp - currentPose.getRotation().getDegrees();
        //temp = temp - ((currentPose.getRotation().getDegrees() + 180.0) % 360.0);

        return temp;
    }

    public double getAngleToTarget() {
        return angleToTarget;
    }

    // private double calculateTurretPosition() {
    //     // Pull data from the limelight to calculate this angle
    //     // If over roated, spin around the other side

    //     if(ShooterConstants.maxTurretPosition > actualTurretPosition && ShooterConstants.minTurretPosition < actualTurretPosition) {
    //         // we should be good, we are within tolerable ranges
    //     } // we need to determine if we need to spin around

    //     return 0.0;
    // }

    public String getStateString() {
        return stateText;
    }

    public void setManualControl(boolean manualControl) {
        this.manualControl = manualControl;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    public void moveLaterally(double moveAmount) {
        targetTurretPosition += moveAmount;
    }
    
}
