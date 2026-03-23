package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.TimerTask;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.node.POJONode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.ServoHubSimFaultManager;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        Tracking,
        Eject,
        ReverseIndexer
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

    public final Translation2d[] targets = {
        new Translation2d(4.5,4.0), // Blue tower
        new Translation2d(12.0, 4.0), // Red tower
        new Translation2d(0.0, 1.0), // Blue Human Element
        new Translation2d(16.6, 7.0), // Red Human Element
        new Translation2d(0.0, 7.0), // Blue Wall
        new Translation2d(16.5, 1.0) // Red Wall 
    };

    private boolean manualControl = false;

    private double[] flywheelPID = ShooterConstants.flywheelPID;

    // Fly wheel motors
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
    private double[] turretPID = ShooterConstants.turretPID;
    public double operatorTurretModifier = 0.0;

    // This is the indexer motor, it spins the indexer around to feed the kicker/turret
    private SparkFlex indexerMotor = null;
    private SparkFlexSim indexerMotorSim = null;

    // These are the kicker motors, that pull the ball up from the indexer and pushed into the turret
    private SparkMax kickMotor1 = null;
    private SparkMaxSim kickMotor1Sim = null;
    private SparkMax kickMotor2 = null;
    private SparkMaxSim kickMotor2Sim = null;


    //private SparkMax hoodMotor = null;
    private double turretTargetAngle = 0.0;

    // These are the hood actuators to lift/lower the hood
    private Servo hoodActuator1 = null;
    private Servo hoodActuator2 = null;
    private boolean hoodOverride = false;

    public double operatorFlywheelVelocityModifier = 0.0;
    private double targetFlyWheelSpeed = 0.0;
    private double actualFlyWheelSpeed = 0.0;
    private boolean flyWheelTargetVelocityOverride = false;

    public double operatorHoodModifier = 0.0;
    private double targetHoodValue = 0.3;
    private double actualHoodValue = 0.0;
    private double targetTurretPosition = 0.0;
    private double actualTurretPosition = 0.0;
    private double turretAngle = 0.0;

    private double distanceFromTarget = 0.0;

    private String stateText = "Stopped";
    public ShooterState shooterState = ShooterState.Tracking;

    private NetworkTableInstance inst = null;
	private NetworkTable table = null;

    private DoubleTopic topicFlywheelVelocity = null;
	private DoublePublisher pubFlywheelVelocity = null;

    private DoubleTopic topicFlywheelTargetVelocity = null;
    private DoublePublisher pubFlywheelTargetVelocity = null;
    private DoubleSubscriber subFlywheelTargetVelocity = null;

    private BooleanTopic topicFlywheelTargetVelocityOverride = null;
    private BooleanPublisher pubFlywheelTargetVelocityOverride = null;
    private BooleanSubscriber subFlywheelTargetVelocityOverride = null;

    private BooleanTopic topicHubIsActive = null;
    private BooleanPublisher pubHubIsActive = null;
    private BooleanSubscriber subHubIsActive = null;

    private DoubleArrayTopic topicFlywheelPID = null;
    private DoubleArrayPublisher pubFlywheelPID = null;
    private DoubleArraySubscriber subFlywheelPID = null;

    private DoubleTopic topicHoodPosition = null;
	private DoublePublisher pubHoodPosition = null;
    private DoubleSubscriber subHoodPosition = null;

    private DoubleTopic topicTurretPosition = null;
	private DoublePublisher pubTurretPosition = null;

    private DoubleTopic topicTurretTargetPosition = null;
	private DoublePublisher pubTurretTargetPosition = null;

    private DoubleArrayTopic topicTurretPID = null;
	private DoubleArrayPublisher pubTurretPID = null;
    private DoubleArraySubscriber subTurretPID = null;

    private DoubleTopic topicDistanceFromTarget = null;
	private DoublePublisher pubDistanceFromTarget = null;

    private StringTopic topicStateString = null;
	private StringPublisher pubStateString = null;

    private StringTopic topicTargetString = null;
	private StringPublisher pubTargetString = null;

    private BooleanTopic topicManualControl = null;
	private BooleanPublisher pubManualControl = null;
    private BooleanSubscriber subManualControl = null;

    private BooleanTopic topicManualHood = null;
    private BooleanPublisher pubManualHood = null;
    private BooleanSubscriber subManualHood = null;

    private DoubleTopic topicHoodAdj = null;
	private DoublePublisher pubHoodAdj = null;
    private DoubleSubscriber subHoodAdj = null;

    private DoubleTopic topicHoodMod = null;
    private DoublePublisher pubHoodMod = null;
    private DoubleSubscriber subHoodMod = null;

    private DoubleTopic topicFlyMod = null;
    private DoublePublisher pubFlyMod = null;
    private DoubleSubscriber subFlyMod = null;

    private AnalogEncoder hoodEncoder = new AnalogEncoder(ShooterConstants.hoodEncoder1Port);
    private AnalogEncoder hoodEncoder2 = new AnalogEncoder(ShooterConstants.hoodEncoder2Port);

    private Limelight limelight = null;
    private int limelightTarget = -1;

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

            // Hood Motor
            hoodActuator1 = new Servo(Constants.ShooterConstants.hoodActuator1Port);
            hoodActuator2 = new Servo(Constants.ShooterConstants.hoodActuator2Port);

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
            kickMotor1 = new SparkMax(ShooterConstants.kickMotor1Id, MotorType.kBrushless);
            SparkFlexConfig kick1Config = new SparkFlexConfig();
            kick1Config
                    .smartCurrentLimit(ShooterConstants.kickMotorCurrentLimit)
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertKickMotor1).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            kickMotor1.configure(kick1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Kick Motor 2
            kickMotor2 = new SparkMax(ShooterConstants.kickMotor2Id, MotorType.kBrushless);
            SparkFlexConfig kick2Config = new SparkFlexConfig();
            kick2Config
                    .smartCurrentLimit(ShooterConstants.kickMotorCurrentLimit)
                    .idleMode(IdleMode.kCoast)
                    //.follow(ShooterConstants.kickMotor1Id);
                    .inverted(ShooterConstants.invertKickMotor2).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            kickMotor2.configure(kick2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Turret Motor
            turretMotor = new SparkMax(ShooterConstants.turretMotorId, MotorType.kBrushless);
            SparkMaxConfig turretConfig = new SparkMaxConfig();

            turretPID = ShooterConstants.turretPID;
            
            turretConfig
                .idleMode(IdleMode.kBrake)
                .inverted(ShooterConstants.invertTurretMotor)
                .smartCurrentLimit(ShooterConstants.turretMotorCurrentLimit)
                .closedLoop
                //.outputRange(-10.0, 10.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                        turretPID[0],
                        turretPID[1],
                        turretPID[2]
                    );
        

            turretConfig.signals.primaryEncoderPositionPeriodMs(5);
            turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            relativeEncoderTurret = (SparkRelativeEncoder) turretMotor.getEncoder();
            
           
            motorPositionTurret = relativeEncoderTurret.getPosition();

            // Setup the network tables info
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("ShooterSubsystem");

            topicFlywheelVelocity = table.getDoubleTopic("Flywheel Velocity");
            pubFlywheelVelocity = topicFlywheelVelocity.publish();
            pubFlywheelVelocity.set(actualFlyWheelSpeed);

            topicFlywheelTargetVelocity = table.getDoubleTopic("Flywheel Target Velocity");
            pubFlywheelTargetVelocity = topicFlywheelTargetVelocity.publish();
            pubFlywheelTargetVelocity.set(targetFlyWheelSpeed);
            subFlywheelTargetVelocity = topicFlywheelTargetVelocity.subscribe(0.0);

            topicFlywheelTargetVelocityOverride = table.getBooleanTopic("Flywheel Target Velocity Override");
            pubFlywheelTargetVelocityOverride = topicFlywheelTargetVelocityOverride.publish();
            pubFlywheelTargetVelocityOverride.set(flyWheelTargetVelocityOverride);
            subFlywheelTargetVelocityOverride = topicFlywheelTargetVelocityOverride.subscribe(false);

            topicFlywheelPID = table.getDoubleArrayTopic("Flywheel PID");
            pubFlywheelPID = topicFlywheelPID.publish();
            pubFlywheelPID.set(ShooterConstants.flywheelPID);
            subFlywheelPID = topicFlywheelPID.subscribe(ShooterConstants.flywheelPID);

            topicHoodPosition = table.getDoubleTopic("Hood Position");
            pubHoodPosition = topicHoodPosition.publish();
            pubHoodPosition.set(actualHoodValue);

            topicTurretPosition = table.getDoubleTopic("Turret Position");
            pubTurretPosition = topicTurretPosition.publish();
            pubTurretPosition.set(actualTurretPosition);

            topicTurretTargetPosition = table.getDoubleTopic("Turret Target Position");
            pubTurretTargetPosition = topicTurretTargetPosition.publish();
            pubTurretTargetPosition.set(actualTurretPosition);

            topicTurretPID = table.getDoubleArrayTopic("Turret PID");
            pubTurretPID = topicTurretPID.publish();
            pubTurretPID.set(turretPID);
            subTurretPID = topicTurretPID.subscribe(ShooterConstants.turretPID);

            topicDistanceFromTarget = table.getDoubleTopic("Distance from Target");
            pubDistanceFromTarget = topicDistanceFromTarget.publish();
            pubDistanceFromTarget.set(distanceFromTarget);

            topicStateString = table.getStringTopic("StateString");
            pubStateString = topicStateString.publish();
            pubStateString.set(stateText);

            topicTargetString = table.getStringTopic("TargetString");
            pubTargetString = topicTargetString.publish();
            pubTargetString.set(shootTargetText);

            topicHoodAdj = table.getDoubleTopic("Hood Target Value");
            pubHoodAdj = topicHoodAdj.publish();
            pubHoodAdj.set(targetHoodValue);
            subHoodAdj = topicHoodAdj.subscribe(targetHoodValue);

            topicManualControl = table.getBooleanTopic("ManualControl");
            pubManualControl = topicManualControl.publish();
            pubManualControl.set(manualControl);
            subManualControl = topicManualControl.subscribe(false);

            topicManualHood = table.getBooleanTopic("Hood Manual Override");
            pubManualHood = topicManualHood.publish();
            pubManualHood.set(hoodOverride);
            subManualHood = topicManualHood.subscribe(false);

            topicHoodMod = table.getDoubleTopic("Operator Hood Modification");
            pubHoodMod = topicHoodMod.publish();
            pubHoodMod.set(operatorHoodModifier);
            subHoodMod = topicHoodMod.subscribe(0.0);

            topicFlyMod = table.getDoubleTopic("Operator Flywheel Velocity Modification");
            pubFlyMod = topicFlyMod.publish();
            pubFlyMod.set(operatorFlywheelVelocityModifier);
            subFlyMod = topicFlyMod.subscribe(0.0);
            
            topicHubIsActive = table.getBooleanTopic("Hub Status");
            pubHubIsActive = topicHubIsActive.publish();
            pubHubIsActive.set(RobotContainer.matchTimeSubsystem.isHubActive());
            

            limelight = new Limelight("limelight");

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
            slot0Configs.kP = Constants.ShooterConstants.flywheelPID[0]; // An error of 1 rps results in 0.11 V output
            slot0Configs.kI = Constants.ShooterConstants.flywheelPID[1]; // no output for integrated error
            slot0Configs.kD = Constants.ShooterConstants.flywheelPID[2]; // no output for error derivative
		    flywheelMotor1.getConfigurator().apply(slot0Configs);

            // Flywheel Motor 2
            flywheelMotor2Config = new TalonFXConfiguration();
		    flywheelMotor2Config
		        .withMotorOutput(
			        new MotorOutputConfigs()
				        .withInverted(InvertedValue.Clockwise_Positive)
				        .withNeutralMode(NeutralModeValue.Coast)
		        );
		    flywheelMotor2Config.Audio.AllowMusicDurDisable = true;
		
		    flywheelMotor2.getConfigurator().apply(flywheelMotor1Config);

		    CurrentLimitsConfigs currentConfig2 = new CurrentLimitsConfigs();
		    currentConfig2.StatorCurrentLimitEnable = true;
		    currentConfig2.StatorCurrentLimit = Constants.ShooterConstants.flywheel2CurrentLimit;
		    flywheelMotor2.getConfigurator().apply(currentConfig2);
		    flywheelMotor2.getConfigurator().apply(slot0Configs);

            // Tell the second fly wheel motor to follow motor1 but in the opposite direction
            flywheelMotor2.setControl(new Follower(ShooterConstants.flywheelMotor1Id, MotorAlignmentValue.Opposed));
        }
    }

    @Override
	public void periodic() {

        if (ShooterConstants.enabled) {

            if(ShooterConstants.debug) {
                modifyCalculations(RobotContainer.driverController.getLeftY(), RobotContainer.operatorController.getRightY());

                // if (hoodActuator1.get() != subHoodAdj.get()){
                //     targetHoodValue = subHoodAdj.get();
                // }
                // if (subManualHood.get()){
                // targetHoodValue = subHoodAdj.get();
                // calculateHood();
                // hoodActuator1.set(targetHoodValue);

                pubHubIsActive.set(RobotContainer.matchTimeSubsystem.isHubActive());
                
                if (subFlyMod.get() != operatorFlywheelVelocityModifier){
                    operatorFlywheelVelocityModifier = subFlyMod.get();
                }
                if (subHoodMod.get() != operatorHoodModifier){
                    operatorHoodModifier = subHoodMod.get();
                }
                pubHoodPosition.set(hoodEncoder.get());
                // }

                // Are we trying to change the turret PID dynamically?
                if(
                    turretPID[0] != subTurretPID.get()[0]
                    || turretPID[1] != subTurretPID.get()[1]
                    || turretPID[2] != subTurretPID.get()[2]
                ) {

                    // We have changed the PID value so lets re-configure the motor with the new values
                    turretPID = subTurretPID.get();

                    SparkMaxConfig turretConfig = new SparkMaxConfig();

                    turretConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ShooterConstants.invertTurretMotor)
                        .smartCurrentLimit(ShooterConstants.turretMotorCurrentLimit)
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(
                            turretPID[0],
                            turretPID[1],
                            turretPID[2]
                        );

                    turretConfig.signals.primaryEncoderPositionPeriodMs(5);
                    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                }
                
            }

            // Set the override
            if(flyWheelTargetVelocityOverride != subFlywheelTargetVelocityOverride.get()) {
                flyWheelTargetVelocityOverride = subFlywheelTargetVelocityOverride.get();
            }

            // Get if we are trying to override the hood location
            if (hoodOverride != subManualHood.get()){
                hoodOverride = subManualHood.get();
            }

            // Set the targetFlywheelSpeed
            if(flyWheelTargetVelocityOverride && targetFlyWheelSpeed != subFlywheelTargetVelocity.get()) {
                targetFlyWheelSpeed = subFlywheelTargetVelocity.get();
            } else {
                // calculateFlywheelSpeed(limelightTarget, target, currentPosition);
                calculateShootOnTheMoveModifiers(limelightTarget);
            }

            if (subFlyMod.get() != operatorFlywheelVelocityModifier){
                operatorFlywheelVelocityModifier = subFlyMod.get();
            }

            if (subHoodMod.get() != operatorHoodModifier){
                operatorHoodModifier = subHoodMod.get();
            }

            // Set the manual control
            if(manualControl != subManualControl.get()) {
                manualControl = subManualControl.get();
            }

            // Are we attempting to change the flywheel PID dynamically?
            if(
                flywheelPID[0] != subFlywheelPID.get()[0]
                || flywheelPID[1] != subFlywheelPID.get()[1]
                || flywheelPID[2] != subFlywheelPID.get()[2]
            ) {
                // we are changing the flywheel PID dynamically
                flywheelPID = subFlywheelPID.get();

                var slot0Configs = new Slot0Configs();
		        slot0Configs.kS = 0.1;
		        slot0Configs.kV = 0.0;
                slot0Configs.kP = flywheelPID[0]; // An error of 1 rps results in 0.11 V output
                slot0Configs.kI = flywheelPID[1]; // no output for integrated error
                slot0Configs.kD = flywheelPID[2]; // no output for error derivative
		        flywheelMotor1.getConfigurator().apply(slot0Configs);
            }

            

            // Get the position from the drive system
            currentPosition = RobotContainer.driveSubsystem.getPoseEstimatorPose2d();

            // we need a function for this to calculate the angle
            turretAngle = RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getRotation().getDegrees();
            // calculateHood();
            calculateShootOnTheMoveModifiers(limelightTarget);
            
            //  change behaviour based off the the state of the shooter subsystem
            switch (shooterState) {
                case Shooting:
                    // The turret is tracking the target, spinning the fly wheel, and we are kicking the fuel
                    // up to the turret to be shot
                    stateText = "Shooting";
                    //indexerMotor.getClosedLoopController().setSetpoint(ShooterConstants.indexMotorSpeed, ControlType.kVelocity);
                    indexerMotor.set(ShooterConstants.indexMotorSpeed);
                    //kickMotor1.getClosedLoopController().setSetpoint(ShooterConstants.kickMotor1Speed, ControlType.kVelocity);
                    //kickMotor1.set(.7);
                    
                    //kickMotor2.getClosedLoopController().setSetpoint(ShooterConstants.kickMotor2Speed, ControlType.kVelocity);

                    // This is for the turret
                    
                    // This is for testing
                    if(Constants.kEnableLimelight && limelight.lookForTarget(limelightTarget)) {
                        // The Limelight is enabled and it sees the target
                        if(limelight.getHorizontalOffsetFromTarget(limelightTarget) > 0) {
                            turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle - 0.1, ControlType.kPosition);
                        } else {
                            turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle + 0.1, ControlType.kPosition);
                        }
                    } else {
                        // turretTargetAngle = (findAngleToTarget(target, currentPosition) * 47)/360; // this is in ticks
            
                    }
                    // Set the turret motor to the position
                    if(turretTargetAngle > -27 || turretTargetAngle < 27) {
                        turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, ControlType.kPosition);
                    } else {
                        System.out.println("turretangle is: " + turretAngle);
                    }
                    
                    

                    // Only need to set flyWheelMotor1 since flyWheelMotor2 follows it
                    //flywheelMotor1.getClosedLoopController().setSetpoint(actualFlyWheelSpeed, ControlType.kVelocity);
                    // if (subFlywheelTargetVelocityOverride.get()){
                    //     targetFlyWheelSpeed = subFlywheelTargetVelocity.get();
                    // }

                    // Set the flywheel to the specific speed needed
                    flywheelMotor1.setControl(new VelocityDutyCycle(targetFlyWheelSpeed));                    
                    if (RobotContainer.driverController.rightBumper().getAsBoolean()){
                        // hoodActuator1.set(0.3);
                    } else {
                        hoodActuator1.set(targetHoodValue);
                        hoodActuator2.set(targetHoodValue);
                        kickMotor1.set(ShooterConstants.kickMotor1Speed);
                        kickMotor2.set(ShooterConstants.kickMotor2Speed);
                    }
                    // calculateHood();
                    // if (subManualHood.get()){
                    //     hoodActuator1.set(subHoodAdj.get());
                    //     java.util.Timer timer = new java.util.Timer();
                    //     TimerTask t = new TimerTask() {

                    //         @Override
                    //         public void run() {
                    //             if (RobotContainer.driverController.rightTrigger().getAsBoolean()){
                    //                 kickMotor1.set(ShooterConstants.kickMotor1Speed);
                    //                 kickMotor2.set(ShooterConstants.kickMotor2Speed);
                    //             } else {
                    //                 kickMotor1.set(0);
                    //                 kickMotor2.set(0);
                    //             }
                                
                    //         }
                            
                    //     };

                    //     timer.schedule(t, 2200);

                    // } else {
                    //     hoodActuator1.set(targetHoodValue);
                    //     pubHoodAdj.set(targetHoodValue);


                        // java.util.Timer timer = new java.util.Timer();
                        // TimerTask t = new TimerTask() {

                        //     @Override
                        //     public void run() {
                        //         if (RobotContainer.driverController.rightTrigger().getAsBoolean() || DriverStation.isAutonomous()){
                        //             kickMotor1.set(ShooterConstants.kickMotor1Speed);
                        //             kickMotor2.set(ShooterConstants.kickMotor2Speed);
                        //         } else {
                        //             kickMotor1.set(0);
                        //             kickMotor2.set(0);
                        //         }
                                
                        //     }
                            
                        // };

                    //     timer.schedule(t, 2200);
                    // }

                    // kickMotor1.set(ShooterConstants.kickMotor1Speed);
                    //kickMotor2.set(.7);
                    // kickMotor2.set(ShooterConstants.kickMotor2Speed);
                    // hoodActuator1.set(targetHoodValue);
                    break;
                case Stopped:
                    // Stop everything
                    stateText = "Stopped";
                    indexerMotor.set(0.0);
                    kickMotor1.set(0.0);
                    kickMotor2.set(0.0);

                    // Stop the flywheel, only need to set flywheelmotor1 because flywheel motor follows it
                    flywheelMotor1.setControl(new VelocityDutyCycle(0.0));
                    pubHoodAdj.set(0);
                    // if (subManualHood.get()){
                    hoodActuator1.set(0.3);
                    // } else {
                        // hoodActuator1.set(0.0);
                    // }

                    break;
                case Tracking:
                    // The turret is tracking the target, spinning the fly wheel, but not spinning/kicking the fuel
                    // up to the turret
                    stateText = "Tracking";
                    indexerMotor.set(0.0);
                    kickMotor1.set(0.0);
                    kickMotor2.set(0.0);

                    if (RobotContainer.driverController.rightBumper().getAsBoolean()){
                        hoodActuator1.set(0.3);
                    } else {
                        hoodActuator1.set(targetHoodValue);
                    }

                    // These are for the turret

                    // This is for testing 
                    // It checks if the limelight can see the target and moves left or right if the target is not in the center of the
                    // field of view.  It also checks if the target position is within the range of movement
                    if(Constants.kEnableLimelight && limelight.lookForTarget(limelightTarget)) {
                        // The Limelight is enabled and it sees the target
                        if(limelight.getHorizontalOffsetFromTarget(limelightTarget) > 0.5  && actualTurretPosition > 0.1) {
                            turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle - 0.1, ControlType.kPosition);
                        } else if(limelight.getHorizontalOffsetFromTarget(limelightTarget) < 0.5 && actualTurretPosition < 44.0) {
                            turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle + 0.1, ControlType.kPosition);
                        }

                        System.out.println("Using limelight to track.");
                    } else {

                        // the limelight is disabled so we are using odometry to calculate 
                        // target angle
                        turretTargetAngle = (findAngleToTarget(target, currentPosition) * 47) / 360; // this is in ticks
                    }

                    // Set the position of the turret motor
                    // turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, ControlType.kPosition);
                     if(turretTargetAngle > -27 || turretTargetAngle < 27) {
                        turretMotor.getClosedLoopController().setSetpoint(turretTargetAngle, ControlType.kPosition);
                    } else {
                        System.out.println("turretangle is: " + turretAngle);
                    }
                    // Only need to set flyWheelMotor1 since flyWheelMotor2 follows it
                    // if (subFlywheelTargetVelocityOverride.get()){
                    //     targetFlyWheelSpeed = subFlywheelTargetVelocity.get();
                    // }
                    flywheelMotor1.setControl(new VelocityDutyCycle(targetFlyWheelSpeed));                    
                    
                    // If we are not shooting, retract the hood to go under the tunnel
                    
                    // hoodActuator1.set(0.3);
                    
                    // if (subManualHood.get()){
                    //     hoodActuator1.set(subHoodAdj.get());

                    // } else {
                    //     hoodActuator1.set(0.0);
                    //     pubHoodAdj.set(0);
                    // }

                    break;
                case ReverseIndexer:
                    stateText = "indexer reversed";
                    indexerMotor.set(-ShooterConstants.indexMotorSpeed);
                    break;
                case Eject:
                    stateText = "Ejecting";
                    indexerMotor.set(-0.5);
                    kickMotor1.set(-0.7);
                    kickMotor2.set(-0.7);
                    // flywheelMotor1.setControl(new VelocityDutyCycle(-1));
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

                        //actualFlyWheelSpeed = calculateFlywheelSpeed(10, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        // targetFlyWheelSpeed = calculateFlywheelSpeed(10, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        calculateShootOnTheMoveModifiers(10);
                        limelightTarget = 10;

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() < 4.0) {
                        shootTarget = ShooterTarget.RedWall;
                        shootTargetText = "Red Wall";
                        target = targets[5];

                        //actualFlyWheelSpeed = calculateFlywheelSpeed(-1, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        targetFlyWheelSpeed = calculateFlywheelSpeed(-1, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        limelightTarget = -1;

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() > 4.0) {
                        shootTarget = ShooterTarget.RedHumanElement;
                        shootTargetText = "Red Human Element";
                        target = targets[3];

                        //actualFlyWheelSpeed = calculateFlywheelSpeed(13, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        targetFlyWheelSpeed = calculateFlywheelSpeed(13, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        limelightTarget = 13;

                    }
                } else if (DriverStation.getAlliance().get() == Alliance.Blue) {

                    // We are on the blue alliance

                    if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getX() < 4.0) {
                        shootTarget = ShooterTarget.BlueTower;
                        shootTargetText = "Blue Tower";
                        target = targets[0];

                        //actualFlyWheelSpeed = calculateFlywheelSpeed(25, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        targetFlyWheelSpeed = calculateFlywheelSpeed(25, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() < 4.0) {
                        shootTarget = ShooterTarget.BlueHumanElement;
                        shootTargetText = "Blue Human Element";
                        target = targets[2];

                        //actualFlyWheelSpeed = calculateFlywheelSpeed(30, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        targetFlyWheelSpeed = calculateFlywheelSpeed(30, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() > 4.0) {
                        shootTarget = ShooterTarget.BlueWall;
                        shootTargetText = "Blue Wall";
                        target = targets[4];

                        //actualFlyWheelSpeed = calculateFlywheelSpeed(-1, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
                        targetFlyWheelSpeed = calculateFlywheelSpeed(-1, target, RobotContainer.driveSubsystem.getPoseEstimatorPose2d());

                    }
                }

                
            }

            // Get where the motors actually are
            actualTurretPosition = relativeEncoderTurret.getPosition();
            //actualTurretPosition = turretMotor.getEncoder().getPosition();

            // Publish to network tables the values
            pubFlywheelVelocity.set(flywheelMotor1.getVelocity().getValueAsDouble());
            pubHoodPosition.set(actualHoodValue);
            
            pubTurretPosition.set(actualTurretPosition);
            pubTurretTargetPosition.set(turretTargetAngle);

            pubDistanceFromTarget.set(distanceFromTarget);
            pubStateString.set(stateText);
            pubTargetString.set(shootTargetText);
            pubManualControl.set(manualControl);
            if (!subhood)){
            pubHoodAdj.set(targetHoodValue);
            }
            pubHoodMod.set(operatorHoodModifier);
            pubFlyMod.set(operatorFlywheelVelocityModifier);

            

            // Show the position on the map
            currentPosition = new Pose2d(
                currentPosition.getX(), 
                currentPosition.getY(), 
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
            kickMotor1Sim = new SparkMaxSim(kickMotor1, DCMotor.getNeoVortex(1));
            kickMotor1Sim.setBusVoltage(12.0);

            // Kick motor 2
            kickMotor2Sim = new SparkMaxSim(kickMotor2, DCMotor.getNeoVortex(1));
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

            // apply the new rotor position and velocity to the TalonFX;
   		    // note that this is rotor position/velocity (before gear ratio), but
   		    // DCMotorSim returns mechanism position/velocity (after gear ratio)
   		    flywheel1SimState.setRawRotorPosition(flywheelMotor1SimModel.getAngularPosition().times(1.0));
   		    flywheel1SimState.setRotorVelocity(flywheelMotor1SimModel.getAngularVelocity().times(1.0));

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

            // apply the new rotor position and velocity to the TalonFX;
   		    // note that this is rotor position/velocity (before gear ratio), but
   		    // DCMotorSim returns mechanism position/velocity (after gear ratio)
   		    flywheel2SimState.setRawRotorPosition(flywheelMotor2SimModel.getAngularPosition().times(1.0));
   		    flywheel2SimState.setRotorVelocity(flywheelMotor2SimModel.getAngularVelocity().times(1.0));

            // Turret motor

            // 1. Get the motor speed (voltage) from the simulation
            double motorSpeed = turretMotorSim.getAppliedOutput();

            // 2. Simulate the movement (e.g., update position based on speed)
            // In a real simulation, you would use physics models here (WPILib) 
            turretMotorSim.setVelocity(motorSpeed * 5676);
            turretMotorSim.setPosition(turretMotorSim.getPosition() + (motorSpeed * 0.1));

            // 3. Update simulation sensors
            turretMotorSim.iterate(motorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate

            // get positions for kicker motors
            kickMotor1Sim.setVelocity(ShooterConstants.kickMotor1Speed * 5676);
            kickMotor1Sim.setPosition(kickMotor1Sim.getPosition() + (ShooterConstants.kickMotor1Speed * 0.1));

            // 3. Update simulation sensors
            //turretMotorSim.iterate(motorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate

            switch (shooterState) {
                case Shooting:
                    stateText = "Shooting";
                    indexerMotorSim.iterate(ShooterConstants.indexMotorSpeed, 12.0, 0.2);
                    kickMotor1Sim.iterate(ShooterConstants.kickMotor1Speed * 5676, 12.0, 0.2);
                    kickMotor2Sim.iterate(ShooterConstants.kickMotor1Speed * 5676, 12.0, 0.2);
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

    private double getLaunchAngle(){
        return hoodActuator1.get() * 1;
    }

    private double calculateTimeOfFlight(){
        double time = 0;
        time = (distanceFromTarget)/(Math.PI*Units.inchesToMeters(4.00)*targetFlyWheelSpeed*Math.cos(Units.degreesToRadians(getLaunchAngle())));
        return time;
    }
    
    private void calculateShootOnTheMoveModifiers(int targetid){
        Pose2d futurePose = RobotContainer.driveSubsystem.estimateFuturePose2d(calculateTimeOfFlight());
        
        turretTargetAngle = (findAngleToTarget(target, futurePose)*47)/360;
        // TODO: test
        // TODO: make a regression for time of flight in relation to distance
        if (!flyWheelTargetVelocityOverride){
            targetFlyWheelSpeed = calculateFlywheelSpeed(targetid, target, futurePose);
        }
        if (!hoodOverride){
            calculateHood(futurePose);
        }
        
    }

    private double calculateDistance(Pose2d a, Pose2d b){
        double c = Math.sqrt(Math.pow(a.getX() - b.getX(), 2.0) + Math.pow(a.getY() - b.getY(), 2.0));
        return c;
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

        switch (shootTarget){
            case RedWall:
                return (( 6.14961*distanceFromTarget+48.41653 + operatorFlywheelVelocityModifier)-8)/7;
                
            case BlueWall: 
                return ((6.14961*distanceFromTarget+48.41653 + operatorFlywheelVelocityModifier)-8)/7;
            case BlueHumanElement: 
                return ((6.14961*distanceFromTarget+48.41653 + operatorFlywheelVelocityModifier)-8)/7;
            case RedHumanElement: 
                return ((6.14961*distanceFromTarget+48.41653 + operatorFlywheelVelocityModifier)-8)/7;
            default:
                return ( 6.14961*distanceFromTarget+48.41653 + operatorFlywheelVelocityModifier)/7;

        }

        // return ( 6.14961*distanceFromTarget+48.41653 + operatorFlywheelVelocityModifier)/7;

        // return distanceFromTarget;
        
    }

        private void calculateHood(Pose2d pose) {
        // This needs to be filled in with a function to calculate the flywheel speed

        // This needs to be replaced with a function or a lookup table
        // targetHoodValue = distanceFromTarget;

        // hoodActuator1.setPosition(targetHoodValue);
        // hoodActuator2.setPosition(targetHoodValue);
        double trg = (0.622613)/(1+Math.pow(Math.E, -(1.56898*calculateDistance(pose, new Pose2d(target.getX(), target.getY(), target.getAngle()))-1.56271)));

        // targetHoodValue = trg;
        actualHoodValue = hoodActuator1.getPosition();
        if (subManualHood.get()){
            targetHoodValue = subHoodAdj.get();
        } else {
            targetHoodValue = trg + operatorHoodModifier;
        }
        
    }

    private void calculateHood() {
        // This needs to be filled in with a function to calculate the flywheel speed

        // This needs to be replaced with a function or a lookup table
        // targetHoodValue = distanceFromTarget;

        // hoodActuator1.setPosition(targetHoodValue);
        // hoodActuator2.setPosition(targetHoodValue);
        // double trg = (0.622613)/(1+Math.pow(Math.E, -(1.56898*distanceFromTarget-1.56271)));

        // // targetHoodValue = trg;
        // actualHoodValue = hoodActuator1.getPosition();
        // if (subManualHood.get()){
        //     targetHoodValue = subHoodAdj.get();
        // } else {
        //     targetHoodValue = trg + operatorHoodModifier;
        // }
        calculateHood(RobotContainer.driveSubsystem.getPoseEstimatorPose2d());
        
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

        // Logic for shooting on the move turret


        temp += operatorTurretModifier;
        if (temp > 180){
            temp -= 360;
        } else if (temp < -180){
            temp += 360;
        }

        return temp;
    }

    public void modifyCalculations(double flywheelVelo, double hood){
        // if (flywheelVelo <0.2 && flywheelVelo > -0.2){
        //     flywheelVelo = 0;
        // }
        // if (hood < 0.2 && hood > -0.2){
        //     hood = 0;
        // }
        // hood *= 0.1;
        // flywheelVelo *= 0.1;
        // operatorFlywheelVelocityModifier += flywheelVelo;
        // operatorHoodModifier += hood;
        // pubFlyMod.set(operatorFlywheelVelocityModifier);
        // pubHoodMod.set(operatorHoodModifier);
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
