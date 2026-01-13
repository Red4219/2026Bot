package frc.robot.subsystems;


import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.mechanisms.SwerveModule;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;
import com.studica.frc.Navx;

public class DriveSubsystem extends SubsystemBase {

  private SwerveModulePosition[] swervePosition;
  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator poseEstimator = null;

  private boolean gyroTurning = false;
	private double targetRotationDegrees;
	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule rearLeft;
	private final SwerveModule rearRight;
	public List<SwerveModule> swerveList;

  private double xSpeed = 0.0;
	private double ySpeed = 0.0;
	private double rot = 0.0;

  // PID controller for gyro turning
	private ProfiledPIDController gyroTurnPidController = null;

  private double driveP = ModuleConstants.kModuleDriveGains.kP;
	private double driveI = ModuleConstants.kModuleDriveGains.kI;
	private double driveD = ModuleConstants.kModuleDriveGains.kD;

  private double turnP = ModuleConstants.kModuleTurningGains.kP;
	private double turnI = ModuleConstants.kModuleTurningGains.kI;
	private double turnD = ModuleConstants.kModuleTurningGains.kD;

  private SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
		new ChassisSpeeds(0, 0, 0)
	);

  private final Navx gyro = new Navx(0);

  /**
	* Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
  * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
  */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
  * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
  * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
  */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(
		Constants.PhotonVisionConstants.visionMeasurementStdDevsX, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsY, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsTheta
	);
  
  public DriveSubsystem() {

    frontLeft = new SwerveModule(
				"FL",
				ModuleConstants.kFrontLeftDriveMotorPort,
				ModuleConstants.kFrontLeftTurningMotorPort,
				ModuleConstants.kFrontLeftTurningEncoderPort,
				ModuleConstants.kFrontLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				// true
				InvertedValue.Clockwise_Positive

			);

		frontRight = new SwerveModule(
				"FR",
				ModuleConstants.kFrontRightDriveMotorPort,
				ModuleConstants.kFrontRightTurningMotorPort,
				ModuleConstants.kFrontRightTurningEncoderPort,
				ModuleConstants.kFrontRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				// true
				InvertedValue.Clockwise_Positive
			);

		rearLeft = new SwerveModule(
				"RL",
				ModuleConstants.kRearLeftDriveMotorPort,
				ModuleConstants.kRearLeftTurningMotorPort,
				ModuleConstants.kRearLeftTurningEncoderPort,
				ModuleConstants.kRearLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				// true
				InvertedValue.Clockwise_Positive
			);

		rearRight = new SwerveModule(
				"RR",
				ModuleConstants.kRearRightDriveMotorPort,
				ModuleConstants.kRearRightTurningMotorPort,
				ModuleConstants.kRearRightTurningEncoderPort,
				ModuleConstants.kRearRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				// true
				InvertedValue.Clockwise_Positive
			);

		swerveList = new ArrayList<SwerveModule>();
		swerveList.add(frontLeft);
		swerveList.add(frontRight);
		swerveList.add(rearLeft);
		swerveList.add(rearRight);

		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition);

		gyroTurnPidController = new ProfiledPIDController(
				DriveConstants.kGyroTurningGains.kP,
				DriveConstants.kGyroTurningGains.kI,
				DriveConstants.kGyroTurningGains.kD,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxTurningVelocityDegrees,
						DriveConstants.kMaxTurningAcceleratonDegrees));

		gyroTurnPidController.enableContinuousInput(-180, 180);
		gyroTurnPidController.setTolerance(DriveConstants.kGyroTurnTolerance);

		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition,
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs
			);
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double xSpeed, double ySpeed, double rot) {

		// Apply deadbands to inputs
		xSpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;
		ySpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;

		rot *= DriveConstants.kMaxRPM;

		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rot = rot;

		

		swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
			//ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().unaryMinus())
			ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
		);
		
		setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
		
		

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("SwerveModuleStates/Setpoints", desiredStates);
		}
	}
	

  public Navx getGyro() {
    return gyro;
  }
}
