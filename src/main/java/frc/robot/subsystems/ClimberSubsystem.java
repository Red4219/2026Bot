package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    public enum ClimbState {
        Climb,
        NoClimb
    }

    public ClimbState climbState = ClimbState.NoClimb;
    private SparkMax climbMotor;
	private SparkMaxSim turningMaxSim = null;
    private double motorPosition = 0.0;
    private double targetPosition = 0.0;
    private SparkAbsoluteEncoder absoluteEncoder = null;
    private String stringState = "No Climb";

    private NetworkTableInstance inst = null;
	private NetworkTable table = null;

    private DoubleTopic topicPosition = null;
	private DoublePublisher pubPosition = null;

    private StringTopic topicStateString = null;
	private StringPublisher pubStateString = null;

    public ClimberSubsystem() {
        climbMotor = new SparkMax(ClimberConstants.climberMotorId, MotorType.kBrushless);

        // Setup the config for the motor
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .inverted(ClimberConstants.invertMotor)
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                ClimberConstants.kP,
                ClimberConstants.kI,
                ClimberConstants.kD
            );

        sparkMaxConfig.signals.primaryEncoderPositionPeriodMs(5);

        // Apply the config to the motor
        climbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Setup the encoder
        absoluteEncoder = climbMotor.getAbsoluteEncoder();
        motorPosition = absoluteEncoder.getPosition();


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
    
    @Override
    public void simulationPeriodic() {

    }

    @Override
	public void periodic() {

        // From the state, get the target position
        if(climbState == ClimbState.Climb) {
            targetPosition = ClimberConstants.targetPositionClimb;
            stringState = "Climb";
        } else if(climbState == ClimbState.NoClimb) {
            targetPosition = ClimberConstants.targetPositionNoClimb;
            stringState = "No Climb";
        }

        // Go to the position based off of the target position
        climbMotor.getClosedLoopController().setSetpoint(targetPosition, ControlType.kPosition);

        // Grab the position
        motorPosition = absoluteEncoder.getPosition();

        // Publish the position to Network tables
        pubPosition.set(motorPosition);

        pubStateString.set(stringState);
    }

    public String getStateString() {
        return stringState;
    }
}
