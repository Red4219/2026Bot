package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterState {
        Stopped,
        Shooting,
        Tracking
    }

    private String stateText = "Stopped";
    private SparkFlex flywheelMotor = null;
    private SparkMax turretMotor = null;
    private SparkMax hoodMotor = null;

    private double targetFlyWheelSpeed = 0.0;
    private double actualFlyWheelSpeed = 0.0;
    private double targetHoodValue = 0.0;
    private double actualHoodValue = 0.0;
    private double targetTurretPosition = 0.0;
    private double actualTurretPosition = 0.0;

    private double distanceFromTarget = 0.0;

    private ShooterState shooterState = ShooterState.Stopped;

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

    private Limelight limelight = null;

    public ShooterSubsystem() {

        if (ShooterConstants.enabled) {

            // Flywheel Motor
            flywheelMotor = new SparkFlex(ShooterConstants.flywheelMotorId, MotorType.kBrushless);

            // Setup the config for the motors
            SparkFlexConfig flyWheelConfig = new SparkFlexConfig();
            flyWheelConfig
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertFlywheelMotor).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ShooterConstants.flywheelP,
                            ShooterConstants.flywheelI,
                            ShooterConstants.flywheelD);

            flyWheelConfig.signals.primaryEncoderPositionPeriodMs(5);

            // Hood Motor
            hoodMotor = new SparkMax(ShooterConstants.hoodMotorId, MotorType.kBrushless);

            // Setup the config for the motors
            SparkFlexConfig hoodConfig = new SparkFlexConfig();
            hoodConfig
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertHoodMotor).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ShooterConstants.hoodP,
                            ShooterConstants.hoodI,
                            ShooterConstants.hoodD);

            hoodConfig.signals.primaryEncoderPositionPeriodMs(5);

            // Turret Motor
            turretMotor = new SparkMax(ShooterConstants.turretMotorId, MotorType.kBrushless);

            // Setup the config for the motors
            SparkFlexConfig turretConfig = new SparkFlexConfig();
            turretConfig
                    .idleMode(IdleMode.kCoast)
                    .inverted(ShooterConstants.invertTurretMotor).closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ShooterConstants.turretP,
                            ShooterConstants.turretI,
                            ShooterConstants.turretD);

            turretConfig.signals.primaryEncoderPositionPeriodMs(5);

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

            limelight = new Limelight("limelight");
        }
    }

    @Override
	public void periodic() {

        if (ShooterConstants.enabled) {

            switch (shooterState) {
                case Shooting:
                    // Calculate Positions
                    targetHoodValue = calculateHood();
                    targetFlyWheelSpeed = calculateFlywheelSpeed();
                    targetTurretPosition = calculateTurretPosition();
                    stateText = "Shooting";
                    break;
                case Stopped:
                    // Stop everything
                    stateText = "Stopped";
                    targetFlyWheelSpeed = 0.0;
                    targetHoodValue = 0.0;
                    targetTurretPosition = 0.0;
                    break;
                case Tracking:
                    // Calculate Positions
                    targetHoodValue = calculateHood();
                    targetFlyWheelSpeed = calculateFlywheelSpeed();
                    targetTurretPosition = calculateTurretPosition();
                    stateText = "Tracking";
                    break;
            }

            // Tell the motors where they need to be
            flywheelMotor.getClosedLoopController().setSetpoint(targetFlyWheelSpeed, ControlType.kVelocity);
            hoodMotor.getClosedLoopController().setSetpoint(targetHoodValue, ControlType.kPosition);
            turretMotor.getClosedLoopController().setSetpoint(targetTurretPosition, ControlType.kPosition);

            // Get where the motors actually are
            actualFlyWheelSpeed = flywheelMotor.getAbsoluteEncoder().getVelocity();
            actualHoodValue = hoodMotor.getAbsoluteEncoder().getPosition();
            actualTurretPosition = turretMotor.getAbsoluteEncoder().getPosition();

            // Publish to network tables the values
            pubFlywheelVelocity.set(actualFlyWheelSpeed);
            pubHoodPosition.set(actualHoodValue);
            pubTurretPosition.set(actualTurretPosition);
            pubDistanceFromTarget.set(distanceFromTarget);
            pubStateString.set(stateText);

        }
    }

    @Override
    public void simulationPeriodic() {
    }

    private double calculateFlywheelSpeed() {

        if(limelight.lookForTarget(9)) {
            // limelight can see the target we are looking for
            distanceFromTarget = limelight.getDistancToTargetFromRobot(9);
        } else {
            distanceFromTarget = 0.0;
        }

        // This needs to be filled in with a function to calculate the flywheel speed
        return 0.0;
    }

    private double calculateHood() {
        // This needs to be filled in with a function to calculate the flywheel speed
        return 0.0;
    }

    private double calculateTurretPosition() {
        // Pull data from the limelight to calculate this angle
        // If over roated, spin around the other side

        if(ShooterConstants.maxTurretPosition > actualTurretPosition && ShooterConstants.minTurretPosition < actualTurretPosition) {
            // we should be good, we are within tolerable ranges
        } // we need to determine if we need to spin around

        return 0.0;
    }

    public String getStateString() {
        return stateText;
    }
    
}
