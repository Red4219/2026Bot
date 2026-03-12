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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
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
        CollapseIntake,
        Eject
    }

    public IntakeState intakeState = IntakeState.Stop;

    private NetworkTableInstance inst = null;
	private NetworkTable table = null;

    private StringTopic topicStateString = null;
	private StringPublisher pubStateString = null;

    private SparkFlex intakeMotor = null;
    private SparkFlexSim intakeMotorSim = null;

    private SparkMax intakeEjectRetractMotor1 = null;
    private SparkMaxSim intakeEjectRetractMotor1Sim = null;
    private SparkMax intakeEjectRetractMotor2 = null;
    private SparkMaxSim intakeEjectRetractMotor2Sim = null;

    private DoubleTopic topicEjectRetractMotorPosition = null;
    private DoublePublisher pubEjectRetractMotorPosition = null;

    private boolean deployed = false;

    public IntakeSubsystem() {

        if (IntakeConstants.enabled) {

            // Setup the network tables info
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("IntakeSubsystem");

            topicStateString = table.getStringTopic("StateString");
            pubStateString = topicStateString.publish();
            pubStateString.set(stringState);

            topicEjectRetractMotorPosition = table.getDoubleTopic("EjectRetractMotorPositiom");
            pubEjectRetractMotorPosition = topicEjectRetractMotorPosition.publish();
            pubEjectRetractMotorPosition.set(0.0);

            // Motor
            intakeMotor = new SparkFlex(IntakeConstants.intakeMotorId, MotorType.kBrushless);

            // Setup the config for the motor
            SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
            sparkFlexConfig
                    .idleMode(IdleMode.kCoast)
                    //.smartCurrentLimit(IntakeConstants.currentLimit)
                    .inverted(IntakeConstants.invertMotor);
                    // .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // .pid(
                    //         IntakeConstants.kP,
                    //         IntakeConstants.kI,
                    //         IntakeConstants.kD);

            sparkFlexConfig.signals.primaryEncoderPositionPeriodMs(5);

            // Apply the config to the motor
            intakeMotor.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // solenoid1 = new Solenoid(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator1Id);
            // solenoid2 = new Solenoid(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator2Id);

            // Intake Retract Motor 1
            intakeEjectRetractMotor1 = new SparkMax(IntakeConstants.intakeEjectRetractMotor1Id, MotorType.kBrushless);

            SparkMaxConfig sparkMaxConfig1 = new SparkMaxConfig();
            sparkMaxConfig1
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(IntakeConstants.intakeEjectRetractMotorCurrentLimit)
                    .inverted(IntakeConstants.invertIntakeEjectRetractMotor1)
                    .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                        IntakeConstants.kIntakeRetractP,
                        IntakeConstants.kIntakeRetractI,
                        IntakeConstants.kIntakeRetractD
                    );

            intakeEjectRetractMotor1.configure(sparkMaxConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Intake Retract Motor 2
            intakeEjectRetractMotor2 = new SparkMax(IntakeConstants.intakeEjectRetractMotor2Id, MotorType.kBrushless);

            SparkMaxConfig sparkMaxConfig2 = new SparkMaxConfig();
            
            sparkMaxConfig2
                .follow(IntakeConstants.intakeEjectRetractMotor1Id)
                .smartCurrentLimit(IntakeConstants.intakeEjectRetractMotorCurrentLimit)
                .inverted(IntakeConstants.invertIntakeEjectRetractMotor2)
                .idleMode(IdleMode.kBrake);
            intakeEjectRetractMotor2.configure(sparkMaxConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
	public void periodic() {

        if (IntakeConstants.enabled) {

            switch (intakeState) {
                case Eject:
                    stringState = "Eject";
                    //intakeMotor.set(-0.5);
                    intakeMotor.set(IntakeConstants.ejectPower);
                    deployed = true;
                    break;
                case Intake:
                    stringState = "Intake";
                    intakeMotor.set(IntakeConstants.intakePower);
                    //intakeMotor.set(0.35);
                    deployed = true;
                    break;
                case Stop:
                    stringState = "Stop";
                    intakeMotor.set(0.0);
                    deployed = true;
                    break;
                case CollapseIntake:
                    stringState = "CollapseIntake";
                    //intakeMotor.set(0.5);
                    intakeMotor.set(IntakeConstants.intakePower);
                    deployed = false;
                    break;
                default:
                    break;
            }

            // Check if we need to retrect or extend
            if(deployed) {
                // System.out.println("deployed");
                // We don't have to set motor 2 beceause it follows motor 1
                intakeEjectRetractMotor1.getClosedLoopController().setSetpoint(IntakeConstants.intakeExtendedPosition, ControlType.kPosition);
            } else {
                // System.out.println("retracted");
                // We don't have to set motor 2 beceause it follows motor 1
                intakeEjectRetractMotor1.getClosedLoopController().setSetpoint(IntakeConstants.intakeRetractedPosition, ControlType.kPosition);
            }

            // Publish the state string of the intake
            pubStateString.set(stringState);
            pubEjectRetractMotorPosition.set(intakeEjectRetractMotor1.getEncoder().getPosition());
        }
    }

    public void simulationInit() {
        if(IntakeConstants.enabled) {
            intakeMotorSim = new SparkFlexSim(intakeMotor, DCMotor.getNEO(1));
            intakeMotorSim.setBusVoltage(12.0);

            //solenoid1Sim = new SolenoidSim(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator1Id);
            //solenoid2Sim = new SolenoidSim(IntakeConstants.controllerId, PneumaticsModuleType.CTREPCM, IntakeConstants.linearActuator2Id);

            intakeEjectRetractMotor1Sim = new SparkMaxSim(intakeEjectRetractMotor1, DCMotor.getNEO(1));
            intakeEjectRetractMotor1Sim.setBusVoltage(12.0);

            intakeEjectRetractMotor2Sim = new SparkMaxSim(intakeEjectRetractMotor2, DCMotor.getNEO(1));
            intakeEjectRetractMotor2Sim.setBusVoltage(12.0);
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

            ///////////////////

            // 1. Get the motor speed (voltage) from the simulation
            double intakeEjectRetractmotorSpeed = intakeEjectRetractMotor1Sim.getAppliedOutput();

            // 2. Simulate the movement (e.g., update position based on speed)
            // In a real simulation, you would use physics models here (WPILib) 
            intakeEjectRetractMotor1Sim.setVelocity(intakeEjectRetractmotorSpeed * 5676);
            intakeEjectRetractMotor1Sim.setPosition(intakeEjectRetractMotor1Sim.getPosition() + (intakeEjectRetractmotorSpeed * 0.1));
            intakeEjectRetractMotor2Sim.setPosition(intakeEjectRetractMotor2Sim.getPosition() + (intakeEjectRetractmotorSpeed * 0.1));

            // 3. Update simulation sensors
            intakeEjectRetractMotor1Sim.iterate(intakeEjectRetractmotorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate
            intakeEjectRetractMotor2Sim.iterate(intakeEjectRetractmotorSpeed * 5676, RoboRioSim.getVInVoltage(), 0.02); // 20ms update rate

        }
    }

    // public void toggleCollapse() {
    //     if (intakeState == intakeState.CollapseIntake){
    //         intakeState = intakeState.Stop;
    //     } else {
    //         intakeState = intakeState.CollapseIntake;
    //     }
    // }

    public String getStateString() {
        return stringState;
    }
}
