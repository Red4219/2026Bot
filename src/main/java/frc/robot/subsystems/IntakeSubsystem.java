package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private String stringState = "Stopped";

    public enum IntakeState {
        Stop,
        Intake,
        Eject
    }

    public IntakeState intakeState = IntakeState.Stop;

    private NetworkTableInstance inst = null;
	private NetworkTable table = null;

    private StringTopic topicStateString = null;
	private StringPublisher pubStateString = null;

    private SparkFlex intakeMotor = null;
    private SparkFlexSim intakeMotorSim = null;

    public IntakeSubsystem() {

        if (IntakeConstants.enabled) {

            // Setup the network tables info
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("IntakeSubsystem");

            topicStateString = table.getStringTopic("StateString");
            pubStateString = topicStateString.publish();
            pubStateString.set(stringState);

            // Motor
            intakeMotor = new SparkFlex(IntakeConstants.intakeMotorId, MotorType.kBrushless);

            // Setup the config for the motor
            SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
            sparkFlexConfig
                    .idleMode(IdleMode.kCoast)
                    .inverted(IntakeConstants.invertMotor).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            IntakeConstants.kP,
                            IntakeConstants.kI,
                            IntakeConstants.kD);

            sparkFlexConfig.signals.primaryEncoderPositionPeriodMs(5);

            if (RobotBase.isReal()) {
                intakeMotorSim = new SparkFlexSim(intakeMotor, DCMotor.getNeoVortex(1));
            }

            // Apply the config to the motor
            intakeMotor.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
	public void periodic() {

        if (IntakeConstants.enabled) {

            switch (intakeState) {
                case Eject:
                    stringState = "Eject";
                    intakeMotor.set(-IntakeConstants.ejectPower);
                    break;
                case Intake:
                    stringState = "Intake";
                    intakeMotor.set(IntakeConstants.intakePower);
                    break;
                case Stop:
                    stringState = "Stop";
                    intakeMotor.set(0.0);
                    break;
            }

            // Publish the state string of the climber
            pubStateString.set(stringState);
        }
    }

    @Override
    public void simulationPeriodic() {
        
    }

    public String getStateString() {
        return stringState;
    }
}
