package frc.robot.subsystems;

import java.util.List;

import com.fasterxml.jackson.databind.node.POJONode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
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
    private double turretAngle = 0.0;

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

    private StringTopic topicTargetString = null;
	private StringPublisher pubTargetString = null;

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

            topicTargetString = table.getStringTopic("TargetString");
            pubTargetString = topicTargetString.publish();
            pubTargetString.set(shootTargetText);

            limelight = new Limelight("limelight");

            //target = aprilTagFieldLayout.getTagPose(10).get().getTranslation().toTranslation2d();
            // Look at the center of the red target
            //target = new Translation2d(12.0, 4.0);
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
            pubTargetString.set(shootTargetText);

            // Check where we are and change the shooter accordingly
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getX() > 11.5) {
                        shootTarget = ShooterTarget.RedTower;
                        shootTargetText = "Red Tower";
                        target = targets[1];
                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() < 4.0) {
                        shootTarget = ShooterTarget.RedWall;
                        shootTargetText = "Red Wall";
                        target = targets[5];
                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() > 4.0) {
                        shootTarget = ShooterTarget.RedHumanElement;
                        shootTargetText = "Red Human Element";
                        target = targets[3];
                    }
                } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getX() < 4.0) {
                        shootTarget = ShooterTarget.BlueTower;
                        shootTargetText = "Blue Tower";
                        target = targets[0];
                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() < 4.0) {
                        shootTarget = ShooterTarget.BlueHumanElement;
                        shootTargetText = "Blue Human Element";
                        target = targets[2];
                    } else if (RobotContainer.driveSubsystem.getPoseEstimatorPose2d().getTranslation().getY() > 4.0) {
                        shootTarget = ShooterTarget.BlueWall;
                        shootTargetText = "Blue Wall";
                        target = targets[4];
                    }
                }
            }

            double angle = findAngleToTarget(target, currentPosition);

            currentPosition = new Pose2d(
                currentPosition.getX(), 
                currentPosition.getY(), 
                new Rotation2d(Math.toRadians(angle))
            );

            field = RobotContainer.field.getObject("My Objects");

            field.setPoses(List.of(
                currentPosition
            ));
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public void setTarget(ShooterTarget target) {

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

    public double findAngleToTarget(Translation2d targett, Pose2d currentPose) {
        
        double temp = Math.atan2(
            targett.getY() - currentPose.getY(),
            targett.getX() - currentPose.getX()
        );

        temp = Math.toDegrees(temp);

        return temp;
    }

    public double getAngleToTarget() {
        return angleToTarget;
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
