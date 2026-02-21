package frc.robot.mechanisms;


import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.PIDGains;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import static edu.wpi.first.units.Units.*;


public class SwerveModule {
	/** Creates a new SwerveModule. */
	//private final CANBus kCANBus = new CANBus();

	private final TalonFX driveMotor;
	private VelocityDutyCycle targetVelo = new VelocityDutyCycle(0);
	private TalonFXSimState talonFXSimState = null;
	private final SparkMax turningMotor;
	private SparkMaxSim turningMaxSim = null;
	private final CANcoder cancoder;
	private final ProfiledPIDController m_turningPIDController;
	public final double angleZero;
	private final String moduleName;
	private Rotation2d _simulatedAbsoluteEncoderRotation2d = null;

	private double m_moduleAngleRadians;
	private Rotation2d m_moduleAngleRotation2d = new Rotation2d();
	//private SwerveModuleState optimizedState;
	private double angularPIDOutput;
	private double angularFFOutput;
	private double turnOutput;
	private boolean isSim = false;
	private ShuffleboardTab swerveTab = null;
	
	private final Distance kWheelRadius = Inches.of(2);
	//private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	SparkMaxConfig turnConfig = null;
	TalonFXConfiguration driveConfig = null;
	
	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	private DCMotorSim m_motorSimModel = null;

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPID,
			PIDGains drivePID,
			boolean invertTurningMotor,
			// boolean invertDriveMotor
			InvertedValue invertDriveMotor
			) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
			_simulatedAbsoluteEncoderRotation2d = new Rotation2d(0.0);
		}

		// Initialize the motors
		driveMotor = new TalonFX(driveMotorChannel, Constants.kCanivoreCANBusName);
		
		turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

		if(isSim) {
			turningMaxSim = new SparkMaxSim(turningMotor, DCMotor.getNEO(1));
		}

		cancoder = new CANcoder(absoluteEncoderPort, Constants.kCanivoreCANBusName);
		cancoder.clearStickyFaults();
		
		driveConfig = new TalonFXConfiguration();
		driveConfig
		.withMotorOutput(
			new MotorOutputConfigs()
				.withInverted(invertDriveMotor)
				.withNeutralMode(NeutralModeValue.Coast)
		);
		driveConfig.Audio.AllowMusicDurDisable = true;
		
		driveMotor.getConfigurator().apply(driveConfig);

		CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
		currentConfig.StatorCurrentLimitEnable = true;
		currentConfig.StatorCurrentLimit = DriveConstants.kDriveMotorCurrentLimit;
		driveMotor.getConfigurator().apply(currentConfig);

		var slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.1;
		slot0Configs.kV = 0.12;
		slot0Configs.kP = drivePID.kP; // An error of 1 rps results in 0.11 V output
		slot0Configs.kI = drivePID.kI; // no output for integrated error
		slot0Configs.kD = drivePID.kD; // no output for error derivative
		driveMotor.getConfigurator().apply(slot0Configs);

		turnConfig = new SparkMaxConfig();

		turnConfig
            .inverted(invertTurningMotor)
			.smartCurrentLimit(DriveConstants.kTurnMotorCurrentLimit)
            .idleMode(IdleMode.kCoast);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				Constants.ModuleConstants.kModuleDriveGains.kP, 
				Constants.ModuleConstants.kModuleDriveGains.kI, 
				Constants.ModuleConstants.kModuleDriveGains.kD
			);
        turnConfig.signals.primaryEncoderPositionPeriodMs(5);
		
        turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_turningPIDController = new ProfiledPIDController(
			angularPID.kP,
			angularPID.kI,
			angularPID.kD,
			new TrapezoidProfile.Constraints( // radians/s?
					2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
					2 * Math.PI * 1200));

		m_turningPIDController.enableContinuousInput(0, Math.toRadians(360));

		if(Constants.kDebugDriveTrain == true) {
			// Swerve tab stuff
			swerveTab = Shuffleboard.getTab("Swerve");
			swerveTab.addDouble(moduleName + " Absolute", this::getAbsoluteHeading);
			swerveTab.addDouble(moduleName + " Meters", this::getDistanceMeters);
			swerveTab.addString(moduleName + " Abs. Status", this::getStatus);
		}
	}

	public void simulationInit() {

		m_motorSimModel = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1),
				0.001,
				1.0
			),
			DCMotor.getKrakenX60Foc(1)
		);
		
		talonFXSimState = driveMotor.getSimState();
   		talonFXSimState.Orientation = ChassisReference.CounterClockwise_Positive;
   		talonFXSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
	}

	public void simulationPeriodic(SwerveModuleState swerveModuleState) {

		
		m_moduleAngleRadians = Math.toRadians(swerveModuleState.angle.getDegrees());
		_simulatedAbsoluteEncoderRotation2d = swerveModuleState.angle;
		

		talonFXSimState = driveMotor.getSimState();

   		// set the supply voltage of the TalonFX
   		talonFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

   		// get the motor voltage of the TalonFX
   		var motorVoltage = talonFXSimState.getMotorVoltageMeasure();

   		// use the motor voltage to calculate new position and velocity
   		// using WPILib's DCMotorSim class for physics simulation
   		m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
   		m_motorSimModel.update(0.020); // assume 20 ms loop time

   		// apply the new rotor position and velocity to the TalonFX;
   		// note that this is rotor position/velocity (before gear ratio), but
   		// DCMotorSim returns mechanism position/velocity (after gear ratio)
   		talonFXSimState.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(1.0));
   		talonFXSimState.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(1.0));

		//talonFXSimState.setRawRotorPosition(m_motorSimModel.getAngularPosition());
		//talonFXSimState.setRotorVelocity(swerveModuleState.speedMetersPerSecond);

		
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return (cancoder.getAbsolutePosition().refresh().getValueAsDouble() * 360);
	}

	public double getDistanceMeters() {

		// return (driveMotor.getPosition(true).getValueAsDouble()*ModuleConstants.kdriveGearRatioL3*ModuleConstants.kwheelCircumference);
		return (driveMotor.getPosition(true).getValueAsDouble()*ModuleConstants.kdriveGearRatioL3*ModuleConstants.kwheelCircumference);
		// return rotationsToMeters(driveMotor.getPosition(true).getValue()).magnitude();

		//
	}

	public SwerveModuleState getState() {

		if(isSim) {
			m_moduleAngleRotation2d = _simulatedAbsoluteEncoderRotation2d;

			return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), 
				m_moduleAngleRotation2d
			);
		}

		return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), 
			//Rotation2d.fromDegrees(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0)
			Rotation2d.fromRadians(cancoder.getAbsolutePosition(true).getValueAsDouble())
		);
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		m_moduleAngleRadians = Math.toRadians(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0);

		if(isSim){
			m_moduleAngleRotation2d = _simulatedAbsoluteEncoderRotation2d;
			return new SwerveModulePosition(getDistanceMeters(), _simulatedAbsoluteEncoderRotation2d);
		}

		m_moduleAngleRotation2d = Rotation2d.fromDegrees(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0);

		return new SwerveModulePosition(getDistanceMeters(), m_moduleAngleRotation2d);
	}

	// Sets the position of the swerve module
	public void setDesiredState(SwerveModuleState desiredState) {
	
		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		desiredState.optimize(m_moduleAngleRotation2d);

		angularPIDOutput = m_turningPIDController.calculate(m_moduleAngleRadians,
			desiredState.angle.getRadians());

		angularFFOutput = turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		turnOutput = angularPIDOutput + angularFFOutput;

		turningMotor.setVoltage(turnOutput);
		
		targetVelo.Velocity = desiredState.speedMetersPerSecond;

		driveMotor.setControl(targetVelo.withSlot(0));
		//driveMotor.setControl(targetVelo);

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getStatorCurrent().getValueAsDouble());
			Logger.recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getDeviceTemp().getValueAsDouble());
			Logger.recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
			Logger.recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());
		}
	}

	// public static double linearVelocityToRevolutionsPerSecond(double linearVelocity, double radius) {
    //     if (radius <= 0) {
    //         throw new IllegalArgumentException("Radius must be a positive value.");
    //     }

    //     // 1. Calculate angular velocity in radians per second
    //     // ω = v / r
    //     //double angularVelocityRadPerSec = linearVelocity / radius;
	// 	return linearVelocity / radius;

    //     // 2. Convert radians per second to revolutions per second
    //     // 1 revolution = 2π radians
    //     //double revolutionsPerSecond = angularVelocityRadPerSec / (2 * Math.PI);

    //     //return revolutionsPerSecond;
    // }

	// public static double metersPerSecondToRevolutionsPerSecond(double metersPerSecond, double radius) {
    //     if (radius <= 0) {
    //         throw new IllegalArgumentException("Radius must be a positive value.");
    //     }
    //     // Circumference of the circle
    //     double circumference = 2 * Math.PI * radius;

    //     // Revolutions per second = (meters per second) / (circumference per revolution)
    //     return metersPerSecond / circumference;
    // }

	public void resetEncoders() {
		//driveMotor.resetSignalFrequencies();
		driveMotor.setPosition(0.0);
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

	double getAngleZero() {
		return this.angleZero;
	}

	String getStatus() {
		return cancoder.getMagnetHealth().getValue().name();
	}

	public void setTurningPID(double p, double i, double d) {
		m_turningPIDController.setPID(p, i, d);
	}

	public void setDrivePID(double p, double i, double d) {
		
		var slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.1;
		slot0Configs.kV = 0.12;
		slot0Configs.kP = p; // An error of 1 rps results in 0.11 V output
		slot0Configs.kI = i; // no output for integrated error
		slot0Configs.kD = d; // no output for error derivative
		driveMotor.getConfigurator().apply(slot0Configs);

		/*driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				p,
				i,
				d
			);*/
	}
	
	private Distance rotationsToMeters(Angle rotations) {
        /* Apply gear ratio to input rotations */
        var gearedRadians = rotations.in(Radians) / Constants.ModuleConstants.kdriveGearRatioL3;
        /* Then multiply the wheel radius by radians of rotation to get distance */
        return this.kWheelRadius.times(gearedRadians);
    }

    private Angle metersToRotations(Distance meters) {
        /* Divide the distance by the wheel radius to get radians */
        var wheelRadians = meters.in(Meters) / this.kWheelRadius.in(Meters);
        /* Then multiply by gear ratio to get rotor rotations */
        return Radians.of(wheelRadians * Constants.ModuleConstants.kdriveGearRatioL3);
    }

    private LinearVelocity rotationsToMetersVel(AngularVelocity rotations) {
        /* Apply gear ratio to input rotations */
        var gearedRotations = rotations.in(RadiansPerSecond) / Constants.ModuleConstants.kdriveGearRatioL3;
        /* Then multiply the wheel radius by radians of rotation to get distance */
        return this.kWheelRadius.per(Second).times(gearedRotations);
    }

    private AngularVelocity metersToRotationsVel(LinearVelocity meters) {
        /* Divide the distance by the wheel radius to get radians */
        var wheelRadians = meters.in(MetersPerSecond) / this.kWheelRadius.in(Meters);
        /* Then multiply by gear ratio to get rotor rotations */
        return RadiansPerSecond.of(wheelRadians * Constants.ModuleConstants.kdriveGearRatioL3);
    }

	public boolean isMoving(){
		return driveMotor.getVelocity().getValueAsDouble() < 0.1 && driveMotor.getVelocity().getValueAsDouble() > -0.1;
	}
}
