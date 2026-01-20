package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.Vision.CameraEnum;
import frc.robot.mechanisms.SwerveModule;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.signals.InvertedValue;
import com.studica.frc.Navx;

public class DriveSubsystem extends SubsystemBase {

	private Vision vision1 = null;
	private Vision vision2 = null;

	private SwerveModulePosition[] swervePosition;
	private SwerveDriveOdometry odometry;
	private SwerveDrivePoseEstimator poseEstimator = null;

	//private EstimatedRobotPose phoneEstimatedRobotPose1;
	//private EstimatedRobotPose phoneEstimatedRobotPose2;

	//private boolean gyroTurning = false;
	//private double targetRotationDegrees;
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
			new ChassisSpeeds(0, 0, 0));

	private final Navx gyro = new Navx(0);

	/**
	 * Standard deviations of model states. Increase these numbers to trust your
	 * model's state estimates less. This
	 * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
	 * meters.
	 */
	private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

	/**
	 * Standard deviations of the vision measurements. Increase these numbers to
	 * trust global measurements from vision
	 * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
	 * radians.
	 */
	private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(
			Constants.PhotonVisionConstants.visionMeasurementStdDevsX,
			Constants.PhotonVisionConstants.visionMeasurementStdDevsY,
			Constants.PhotonVisionConstants.visionMeasurementStdDevsTheta);

	NetworkTableInstance inst = null;
	NetworkTable table = null;

	DoubleTopic topicDriveP = null;
	DoubleSubscriber subDriveP = null;
	DoublePublisher pubDriveP = null;

	DoubleTopic topicDriveI = null;
	DoubleSubscriber subDriveI = null;
	DoublePublisher pubDriveI = null;

	DoubleTopic topicDriveD = null;
	DoubleSubscriber subDriveD = null;
	DoublePublisher pubDriveD = null;

	DoubleTopic topicTurnP = null;
	DoubleSubscriber subTurnP = null;
	DoublePublisher pubTurnP = null;

	DoubleTopic topicTurnI = null;
	DoubleSubscriber subTurnI = null;
	DoublePublisher pubTurnI = null;

	DoubleTopic topicTurnD = null;
	DoubleSubscriber subTurnD = null;
	DoublePublisher pubTurnD = null;

	public DriveSubsystem() {

		if (Constants.kEnablePhotonVision) {

			if (Constants.kEnablePhotonVisionCamera1) {
				vision1 = new Vision(this::addVisionMeasurement, CameraEnum.Camera1);
			}

			if (Constants.kEnablePhotonVisionCamera2) {
				vision2 = new Vision(this::addVisionMeasurement, CameraEnum.Camera2);
			}

		}

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
				InvertedValue.Clockwise_Positive);

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
				InvertedValue.Clockwise_Positive);

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
				InvertedValue.Clockwise_Positive);

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
				// gyro.getRotation2d().unaryMinus(),
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
				// gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition,
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs);

		gyro.resetYaw();

		if (Constants.kDebugDriveTrain) {

			inst = NetworkTableInstance.getDefault();
			table = inst.getTable("DriveSubsystem");

			topicDriveP = table.getDoubleTopic("driveP");
			subDriveP = topicDriveP.subscribe(0.0);
			pubDriveP = topicDriveP.publish();

			topicDriveI = table.getDoubleTopic("driveI");
			subDriveI = topicDriveI.subscribe(0.0);
			pubDriveI = topicDriveI.publish();

			topicDriveD = table.getDoubleTopic("driveD");
			subDriveD = topicDriveD.subscribe(0.0);
			pubDriveD = topicDriveD.publish();

			topicTurnP = table.getDoubleTopic("turnP");
			subTurnP = topicTurnP.subscribe(0.0);
			pubTurnP = topicTurnP.publish();

			topicTurnI = table.getDoubleTopic("turnI");
			subTurnI = topicTurnI.subscribe(0.0);
			pubTurnI = topicTurnI.publish();

			topicTurnD = table.getDoubleTopic("turnD");
			subTurnD = topicTurnD.subscribe(0.0);
			pubTurnD = topicTurnD.publish();

			
			pubDriveP.set(driveP);
			pubDriveI.set(driveI);
			pubDriveD.set(driveD);

			pubTurnP.set(turnP);
			pubTurnI.set(turnI);
			pubTurnD.set(turnD);
		}
	}

	@Override
	public void periodic() {
		updateOdometry();

		if (Constants.kDebugDriveTrain) {

			if (subDriveP.get() != driveP) {
				driveP = subDriveP.get();
			}

			if (subDriveI.get() != driveI) {
				driveI = subDriveI.get();
			}

			if (subDriveD.get() != driveD) {
				driveD = subDriveD.get();
			}

			if (subTurnP.get() != turnP) {
				turnP = subTurnP.get();
			}

			if (subTurnI.get() != turnI) {
				turnI = subTurnI.get();
			}

			if (subDriveD.get() != driveD) {
				turnD = subTurnD.get();
			}
		}

		//System.out.println("The value is: " + subDriveP.get());
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation

		

		if (vision1.getCamera1Enabled()) {
				vision1.simulationPeriodic(RobotContainer.driveSubsystem.poseEstimator.getEstimatedPosition());
		}

		if (vision2.getCamera1Enabled()) {
				vision2.simulationPeriodic(RobotContainer.driveSubsystem.poseEstimator.getEstimatedPosition());
		}
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
				// ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
				// gyro.getRotation2d().unaryMinus())
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()));

		setModuleStates(swerveModuleStates);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);

		if (Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("SwerveModuleStates/Setpoints", desiredStates);
		}
	}

	public Navx getGyro() {
		return gyro;
	}

	/**
	 * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
	 */
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}

	/**
	 * See
	 * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
	 */
	public void addVisionMeasurement(
			Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	public void updateOdometry() {
		swervePosition[0] = frontLeft.getPosition();
		swervePosition[1] = frontRight.getPosition();
		swervePosition[2] = rearLeft.getPosition();
		swervePosition[3] = rearRight.getPosition();

		if(Robot.isSimulation()) {

			odometry.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition);

			poseEstimator.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition
			);

		} else {
			odometry.update(
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition
			);

			/*estimatedPose = poseEstimator.update(
				gyro.getRotation2d().unaryMinus(),
				swervePosition
			);*/
			poseEstimator.update(
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition
			);
		}

		if (Constants.kEnablePhotonVision) {

			if (vision1.getCamera1Enabled()) {
				vision1.periodic();
			}

			if (vision2.getCamera2Enabled()) {
				vision2.periodic();
			}

			/*if (!gyro.isMoving() && vision1.getCamera1Enabled() && Constants.kResetOdometryFromPhotonVision && !isSim
					&& vision1.isVisionEstAvailable()) {
				resetOdometry(vision1.getEstimatedRobotPose().estimatedPose.toPose2d());
			} else if (!gyro.isMoving() && vision2.getCamera2Enabled() && Constants.kResetOdometryFromPhotonVision
					&& !isSim && vision2.isVisionEstAvailable()) {
				resetOdometry(vision2.getEstimatedRobotPose().estimatedPose.toPose2d());
			}*/

		}

		// Show the estimated position
		Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());

		/*if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
		}*/

		// Update the field with the location of the robot
		//RobotContainer.field.setRobotPose(odometry.getPoseMeters());
		RobotContainer.field.setRobotPose(poseEstimator.getEstimatedPosition());

		
	}

	public void resetOdometry(Pose2d pose) {

		odometry.resetPosition(
			gyro.getRotation2d(),
			swervePosition,
			pose
		);

		poseEstimator.resetPosition(
			gyro.getRotation2d(),
			swervePosition,
			pose
		);
	}

	public double getHeading() {
		if(Robot.isSimulation()) {
			//System.out.println("getHeading is: " + gyro.getRotation2d().getDegrees());
			return gyro.getRotation2d().getDegrees();
		}
		//return gyro.getRotation2d().unaryMinus().getDegrees();
		return gyro.getRotation2d().getDegrees();
	}
}
