package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    public enum ClimbState {
        Stored,
        Up,
        Down
    }

    public ClimbState climbState = ClimbState.Stored;
    private SparkMax climbMotor1 = null;
	private SparkMaxSim climbMax1Sim = null;
    private SparkMax climbMotor2 = null;
	private SparkMaxSim climbMax2Sim = null;
    private DCMotor maxGearbox = null;
    private double motorPosition = 0.0;
    private double targetPosition = 0.0;
    private SparkRelativeEncoder relativeEncoder = null;
    private SparkRelativeEncoderSim relativeEncoderSim = null;
    private String stringState = "Stored";

    private NetworkTableInstance inst = null;
	private NetworkTable table = null;

    private DoubleTopic topicPosition = null;
	private DoublePublisher pubPosition = null;

    private StringTopic topicStateString = null;
	private StringPublisher pubStateString = null;

    public ClimberSubsystem() {

        if (ClimberConstants.enabled) {

            climbMotor1 = new SparkMax(ClimberConstants.climberMotor1Id, MotorType.kBrushless);
            climbMotor2 = new SparkMax(ClimberConstants.climberMotor2Id, MotorType.kBrushless);

            // Setup the config for the motor
            SparkMaxConfig sparkMaxConfig1 = new SparkMaxConfig();
            sparkMaxConfig1
                    .smartCurrentLimit(ClimberConstants.climberMotorCurrentLimit)
                    .idleMode(IdleMode.kBrake)
                    .inverted(ClimberConstants.invertMotor1).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ClimberConstants.kP,
                            ClimberConstants.kI,
                            ClimberConstants.kD);

            SparkMaxConfig sparkMaxConfig2 = new SparkMaxConfig();
            sparkMaxConfig2
                    .smartCurrentLimit(ClimberConstants.climberMotorCurrentLimit)
                    .idleMode(IdleMode.kBrake)
                    .follow(ClimberConstants.climberMotor1Id)
                    .inverted(ClimberConstants.invertMotor2).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ClimberConstants.kP,
                            ClimberConstants.kI,
                            ClimberConstants.kD);

            sparkMaxConfig1.signals.primaryEncoderPositionPeriodMs(5);

            // Apply the config to the motors
            climbMotor1.configure(sparkMaxConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            climbMotor2.configure(sparkMaxConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


            // Setup the encoder
            relativeEncoder = (SparkRelativeEncoder) climbMotor1.getEncoder();
            motorPosition = relativeEncoder.getPosition();

            // Setup the network tables info
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("ClimberSubsystem");

            topicPosition = table.getDoubleTopic("position");
            pubPosition = topicPosition.publish();
            pubPosition.set(motorPosition);

            topicStateString = table.getStringTopic("StateString");
            pubStateString = topicStateString.publish();
            pubStateString.set(stringState);
        }
    }

    public void simulationInit() {
        if (ClimberConstants.enabled) {
            maxGearbox = DCMotor.getNEO(1);
            climbMax1Sim = new SparkMaxSim(climbMotor1, maxGearbox);
            relativeEncoderSim = climbMax1Sim.getRelativeEncoderSim();

            climbMax2Sim = new SparkMaxSim(climbMotor2, maxGearbox);
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (ClimberConstants.enabled) {
            climbMax1Sim.iterate(climbMotor1.getOutputCurrent(), RoboRioSim.getVInVoltage(), 0.02);
            climbMax2Sim.iterate(climbMotor1.getOutputCurrent(), RoboRioSim.getVInVoltage(), 0.02);
        }
    }

    @Override
	public void periodic() {

        if (ClimberConstants.enabled) {

            // From the state, get the target position
            if (climbState == ClimbState.Up) {
                targetPosition = ClimberConstants.targetPositionUp;
                stringState = "Up";
            } else if (climbState == ClimbState.Down) {
                targetPosition = ClimberConstants.targetPositionDown;
                stringState = "Down";
            } else if (climbState == ClimbState.Stored) {
                targetPosition = ClimberConstants.targetPositionStored;
                stringState = "Stored";
            }

            // Go to the position based off of the target position
            climbMotor1.getClosedLoopController().setSetpoint(targetPosition, ControlType.kPosition);

            // Grab the position
            motorPosition = relativeEncoder.getPosition();

            // Publish the position to Network tables
            pubPosition.set(motorPosition);

            // Publish the state string of the climber
            pubStateString.set(stringState);

        }
    }

    public String getStateString() {
        return stringState;
    }

    public boolean atTargetPosition() {

        if(Math.abs(relativeEncoder.getPosition() - targetPosition) < 0.5) {
            return true;
        }

        return false;
    }
}
