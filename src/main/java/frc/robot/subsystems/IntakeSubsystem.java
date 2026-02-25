package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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

    private BooleanTopic topicLinearActuators = null;
    private BooleanPublisher pubLinearActuators = null;

    private SparkFlex intakeMotor = null;
    private SparkFlexSim intakeMotorSim = null;

    // private Solenoid solenoid1 = null;
    // private SolenoidSim solenoid1Sim = null;
    // private Solenoid solenoid2 = null;
    // private SolenoidSim solenoid2Sim = null;

    private SparkMax intakeEjectRetractMotor = null;
    private SparkMaxSim intakeEjectRetractMotorSim = null;

    private boolean deployed = false;

    public IntakeSubsystem() {

        if (IntakeConstants.enabled) {

            // Setup the network tables info
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("IntakeSubsystem");

            topicStateString = table.getStringTopic("StateString");
            pubStateString = topicStateString.publish();
            pubStateString.set(stringState);

            topicLinearActuators = table.getBooleanTopic("LinearActuators");
            pubLinearActuators = topicLinearActuators.publish();
            pubLinearActuators.set(deployed);

            // Motor
            intakeMotor = new SparkFlex(IntakeConstants.intakeMotorId, MotorType.kBrushless);

            // Setup the config for the motor
            SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
            sparkFlexConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(IntakeConstants.currentLimit)
                    .inverted(IntakeConstants.invertMotor).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            IntakeConstants.kP,
                            IntakeConstants.kI,
                            IntakeConstants.kD);

            sparkFlexConfig.signals.primaryEncoderPositionPeriodMs(5);

            /*if (RobotBase.isReal()) {
                intakeMotorSim = new SparkFlexSim(intakeMotor, DCMotor.getNeoVortex(1));
            }*/

            // Apply the config to the motor
            intakeMotor.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // solenoid1 = new Solenoid(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator1Id);
            // solenoid2 = new Solenoid(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator2Id);

            intakeEjectRetractMotor = new SparkMax(IntakeConstants.intakeEjectRetractMotorId, MotorType.kBrushless);

            SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
            sparkMaxConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(IntakeConstants.intakeEjectRetractMotorIdCurrentLimit)
                    .inverted(IntakeConstants.invertIntakeEjectRetractMotor)
                    .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                        IntakeConstants.kIntakeRetractP,
                        IntakeConstants.kIntakeRetractI,
                        IntakeConstants.kIntakeRetractD
                    );

            intakeEjectRetractMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
	public void periodic() {

        if (IntakeConstants.enabled) {

            switch (intakeState) {
                case Eject:
                    stringState = "Eject";
                    intakeMotor.set(-IntakeConstants.ejectPower);

                    deployed = true;

                    break;
                case Intake:
                    stringState = "Intake";
                    intakeMotor.set(IntakeConstants.intakePower);

                    deployed = true;

                    break;
                case Stop:
                    stringState = "Stop";
                    intakeMotor.set(0.0);

                    deployed = false;

                    break;
            }

            //solenoid1.set(deployed);
            //solenoid2.set(deployed);

            if(deployed) {
                intakeEjectRetractMotor.getClosedLoopController().setSetpoint(IntakeConstants.intakeExtendedPosition, ControlType.kPosition);
            } else {
                intakeEjectRetractMotor.getClosedLoopController().setSetpoint(IntakeConstants.intakeRetractedPosition, ControlType.kPosition);
            }

            // Publish the state string of the climber
            pubStateString.set(stringState);
            pubLinearActuators.set(deployed);
        }
    }

    public void simulationInit() {
        if(IntakeConstants.enabled) {
            intakeMotorSim = new SparkFlexSim(intakeMotor, DCMotor.getNEO(1));
            intakeMotorSim.setBusVoltage(12.0);

            //solenoid1Sim = new SolenoidSim(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator1Id);
            //solenoid2Sim = new SolenoidSim(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator2Id);

            intakeEjectRetractMotorSim = new SparkMaxSim(intakeEjectRetractMotor, DCMotor.getNEO(1));
            intakeEjectRetractMotorSim.setBusVoltage(12.0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if(IntakeConstants.enabled) {
            //intakeMotorSim.iterate(intakeMotor.getOutputCurrent(), RoboRioSim.getVInVoltage(), 0.2);

            // 1. Get the motor speed (voltage) from the simulation
            double motorSpeed = intakeMotorSim.getAppliedOutput();

            // 2. Simulate the movement (e.g., update position based on speed)
            // In a real simulation, you would use physics models here (WPILib) 
            intakeMotorSim.setVelocity(motorSpeed * 5676);
            intakeMotorSim.setPosition(intakeMotorSim.getPosition() + (motorSpeed * 0.1));

            // 3. Update simulation sensors
            intakeMotorSim.iterate(motorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate

            //solenoid1Sim.setOutput(deployed);
            //solenoid2Sim.setOutput(deployed);


            ///////////////////

            // 1. Get the motor speed (voltage) from the simulation
            double intakeEjectRetractmotorSpeed = intakeEjectRetractMotorSim.getAppliedOutput();

            // 2. Simulate the movement (e.g., update position based on speed)
            // In a real simulation, you would use physics models here (WPILib) 
            intakeEjectRetractMotorSim.setVelocity(intakeEjectRetractmotorSpeed * 5676);
            intakeEjectRetractMotorSim.setPosition(intakeEjectRetractMotorSim.getPosition() + (intakeEjectRetractmotorSpeed * 0.1));

            // 3. Update simulation sensors
            intakeEjectRetractMotorSim.iterate(intakeEjectRetractmotorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate
        }
    }

    public String getStateString() {
        return stringState;
    }
}
