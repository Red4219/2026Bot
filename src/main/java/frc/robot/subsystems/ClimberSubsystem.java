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
    private SparkMax climbMotor = null;
	private SparkMaxSim climbMaxSim = null;
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

            climbMotor = new SparkMax(ClimberConstants.climberMotorId, MotorType.kBrushless);

            // Setup the config for the motor
            SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
            sparkMaxConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(ClimberConstants.invertMotor).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ClimberConstants.kP,
                            ClimberConstants.kI,
                            ClimberConstants.kD);

            sparkMaxConfig.signals.primaryEncoderPositionPeriodMs(5);

            // Apply the config to the motor
            climbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Setup the encoder
            relativeEncoder = (SparkRelativeEncoder) climbMotor.getEncoder();
            motorPosition = relativeEncoder.getPosition();

            if (RobotBase.isSimulation()) {
                maxGearbox = DCMotor.getNEO(1);
                climbMaxSim = new SparkMaxSim(climbMotor, maxGearbox);
                relativeEncoderSim = climbMaxSim.getRelativeEncoderSim();
            }

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
    
    @Override
    public void simulationPeriodic() {
        climbMaxSim.iterate(climbMotor.getOutputCurrent(), RoboRioSim.getVInVoltage(), 0.02);
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
            climbMotor.getClosedLoopController().setSetpoint(targetPosition, ControlType.kPosition);

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
