// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	// Drive train
	public static final boolean kDebugDriveTrain = true;
	public static final boolean kEnableDriveSubSystemLogger = true;
	public static final boolean enableLogger = true;

	// Photonvision
	public static final boolean kEnablePhotonVision = true;
	public static final boolean kDebugPhotonVision = true;
	public static final boolean kResetOdometryFromPhotonVision = true;
	public static final boolean kEnablePhotonVisionCamera1 = true;
	public static final boolean kEnablePhotonVisionCamera2 = true;

	// Limelight
	public static final boolean kEnableLimelight = false;
	public static final boolean kDebugLimelight = false;

	public static final String kRioCANBusName = "rio";
	public static final String kCanivoreCANBusName = "canivore";
	//public static final String logFolders = "/media/sda2/";

	public static class ModuleConstants {

		public static final String[] song = {"EvangelionMisato.chrp", "EvangelionCAT.chrp", "FetyWap.chrp"}; 
		public static final Boolean enableMusic = false;

		// Current limits for the wheels
		//public static final int kTurnMotorCurrentLimit = 25;
		//public static final int kDriveMotorCurrentLimit = 35;

		// Constants set for the _SDS MK4i_ and MK4
		//public static final double kdriveGearRatioL1 = 1d / 8.14;
		//public static final double kdriveGearRatioL2 = 1d / 6.75;
		//public static final double kdriveGearRatioL3 = 1d / 6.12;
		public static final double kdriveGearRatioL3 = 1d / 5.36;
		//public static final double kdriveGearRatioL3 = 5.36 / 1.0;
		// public static final double kdriveGearRatioL3 = 1d / 5.7;
		//public static final double kdriveGearRatioL4 = 1d / 5.14;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;
		//public static final double kwheelCircumference = 2 * Math.PI * Units.inchesToMeters(2);

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = 20.0;

		//public static final double ksVolts = .1;
		//public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; 

		// Kraken drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 6;
		public static final int kFrontRightDriveMotorPort = 8;
		public static final int kRearLeftDriveMotorPort = 4;
		public static final int kRearRightDriveMotorPort = 2;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 5;
		public static final int kFrontRightTurningMotorPort = 7;
		public static final int kRearLeftTurningMotorPort = 3;
		public static final int kRearRightTurningMotorPort = 1;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 11;
		public static final int kFrontRightTurningEncoderPort = 12;
		public static final int kRearLeftTurningEncoderPort = 10;
		public static final int kRearRightTurningEncoderPort = 9;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = 0.0;
		public static final double kFrontRightAngleZero = 0.0;
		public static final double kRearLeftAngleZero = 0.0;
		public static final double kRearRightAngleZero = 0.0;

		public static final PIDGains kModuleDriveGains = new PIDGains(0.01, 0, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(5.5, 0.0, 0.0);
	}

	public static class DriveConstants {
		// this sets turning speed (keep this low)
		public static final double kMaxRPM = 10;
		public static final double kBumperToBumperWidth = Units.inchesToMeters(27);

		public static final double kTrackWidth = Units.inchesToMeters(27); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(27); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR
		 
		public static final double kGyroYawOffset = 180;
		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

		public static final double kAutoAlignSpeed = 0.02;
		public static final double kAutoAlignTolerance = 1.0;
		public static final double kAutoAlignOffset = 4.81;
		public static final double kAutoAlignRightOffset = 4.81;

		public static final double kAutoAlignLeftLeft = 10.7;
		public static final double kAutoAlignLeftRight = 9.7;

		public static final double kAutoAlignRightLeft = 0.0;
		public static final double kAutoAlignRightRight = 0.0;

		// These are to measure the distnace to the target
		public static final double kAutoAlignLeftYOffset = 25.0;
		public static final double kAutoAlignLeftYTolerance = 4.0;

		public static final int kTurnMotorCurrentLimit = 20;
		public static final double kDriveMotorCurrentLimit = 40;

	}


	/**
	 * The constants pertaining to Autonoumus 
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			//public static final PIDConstants kPPDriveConstants = new PIDConstants(8.5, 0, 0);
			//public static final PIDConstants kPPDriveConstants = new PIDConstants(5.0, 0, 0);
			//public static final PIDConstants kPPDriveConstants = new PIDConstants(6.5, 0, 0); // best one
			public static final PIDConstants kPPDriveConstants = new PIDConstants(6.01, 0.0, 0);
			//public static final PIDConstants kPPTurnConstants = new PIDConstants(3.5, 0, 0);
			//public static final PIDConstants kPPTurnConstants = new PIDConstants(5.0, 0, 0);
			public static final PIDConstants kPPTurnConstants = new PIDConstants(3.5, 0, 0);

			//public static final double kPPMaxVelocity = 4.00;
			//public static final double kPPMaxAcceleration = 2.50;
			//public static final double kMaxModuleSpeed = 4.5; // Max module speed, in m/s
			//public static final double kDriveBaseRadius = 0.4; // Drive base radius in meters. Distance from robot center to furthest module.
		}
	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {

		public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	public static class LimelightConstants {
		public static final String name1 = "limelight";
	}

	public static class PhotonVisionConstants {

		public static final boolean debugPhotonVision = true;
		public static final double camDiagFOV = 170.0;
		public static final double camPitch = 0.0;
		public static final double cam2Pitch = 0.0;
		public static final double camHeightOffGround = Units.inchesToMeters(10.25);
		public static final double cam2HeightOffGround = Units.inchesToMeters(10.25);
		// the side to side position of the camera relative to the robot center
		public static final double camX = 0;
		public static final double cam2X = 0;
		// the front to back position of the camera relative to the robot center
		public static final double camY = -.5;
		public static final double cam2Y = 0.3;

		public static final double camRotation = Math.toRadians(-90.0);
		public static final double cam2Rotation = Math.toRadians(90.0);

		public static final Transform3d cameraToRobot = new Transform3d(
                    new Translation3d(
						camX,
						camY,
					 	PhotonVisionConstants.camHeightOffGround
					),
					new Rotation3d(
						0,
						PhotonVisionConstants.camPitch,
						0
					)
				);
		public static final Transform3d camera2ToRobot = new Transform3d(
                    new Translation3d(
						cam2X,
						cam2Y,
					 	PhotonVisionConstants.cam2HeightOffGround
					),
					new Rotation3d(
						0,
						PhotonVisionConstants.cam2Pitch,
						cam2Rotation
					)
				);

		public static final String CameraName = "cam1";
		public static final String Camera2Name = "cam2";

		// Simulated Vision System.
    	// Configure these to match your PhotonVision Camera,
    	// pipeline, and LED setup.
		public static final double sim_camDiagFOV = camDiagFOV; // degrees - assume wide-angle camera
		public static final double sim_camPitch = camPitch; // degrees
    	public static final double sim_camHeightOffGround = camHeightOffGround; // meters
    	//public static double sim_maxLEDRange = 20; // meters
    	public static final int sim_camResolutionWidth = 640; // pixels
    	public static final int sim_camResolutionHeight = 480; // pixels
    	//public static double sim_minTargetArea = 10; // square pixels
		//public static double sim_minTargetArea = 300; // square pixels

		/**
    	* Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    	* less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    	*/
		public static final double visionMeasurementStdDevsX = 0.5;
		public static final double visionMeasurementStdDevsY = 0.5;
		public static final double visionMeasurementStdDevsTheta = Units.degreesToRadians(10);

		// The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
	}

	public static class ClimberConstants {
		public static final boolean enabled = false;
		public static final int climberMotor1Id = 20;
		public static final int climberMotor2Id = 21;
		public static final boolean invertMotor1 = false;
		public static final boolean invertMotor2 = false;
		public static final int climberMotorCurrentLimit = 10;
		public static final double kP = 5.2;
		public static final double kI = 0.0;
		public static final double kD = 0.0;

		public static final double targetPositionUp = 15.0;
		public static final double targetPositionDown = 10.0;
		public static final double targetPositionStored = 0.0;
	}

	public static class IntakeConstants {
		public static final boolean enabled = false;
		public static final int intakeMotorId = 22;
		public static final boolean invertMotor = false;
		public static final int currentLimit = 10;
		public static final double kP = 0.2;
		public static final double kI = 0.0;
		public static final double kD = 0.0;

		public static final double intakePower = 1.0;
		public static final double ejectPower = 1.0;

		//public static int linearActuator1Id = 0;
		//public static int linearActuator2Id = 1;
		//public static int controllerId = 36;
		public static final int intakeEjectRetractMotor1Id = 36;
		public static final boolean invertIntakeEjectRetractMotor1 = false;
		public static final int intakeEjectRetractMotorCurrentLimit = 10;
		public static final double kIntakeRetractP = 0.2;
		public static final double kIntakeRetractI = 0.0;
		public static final double kIntakeRetractD = 0.0;
		public static final double intakeExtendedPosition = 100;
		public static final double intakeRetractedPosition = 0;

		public static final int intakeEjectRetractMotor2Id = 37;
		public static final boolean invertIntakeEjectRetractMotor2 = false;
	}

	// public static class PoseDefinitions {
	// 	public static enum kFieldPoses {
	// 		APRILTAG_3,
	// 		APRILTAG_4,
	// 		APRILTAG_9,
	// 		APRILTAG_10
	// 	}

	// 	/*public static final Pose2d kProcessorPoseRed = new Pose2d(14.73, 7.69, Rotation2d.fromDegrees(90.0));
	// 	public static final Pose2d kProcessorPoseBlue = new Pose2d(5.973, 0.672, Rotation2d.fromDegrees(115.655));
	// 	public static final Pose2d kReefPoseRed = new Pose2d(0.98, 1.05, Rotation2d.fromDegrees(-120.16));
	// 	public static final Pose2d kReefPoseBlue = new Pose2d(15.35, 0.88, Rotation2d.fromDegrees(-120.0));*/

	// 	/*public static final Pose2d APRILTAG_3 = new Pose2d(5.973, 0.672, Rotation2d.fromDegrees(-90));
	// 	public static final Pose2d APRILTAG_4 = new Pose2d(8.5, 2.0, Rotation2d.fromDegrees(0));
	// 	public static final Pose2d APRILTAG_9 = new Pose2d(5.4, 2.9, Rotation2d.fromDegrees(110));
	// 	public static final Pose2d APRILTAG_10 = new Pose2d(10.973, 0.672, Rotation2d.fromDegrees(-90));*/
	// }

	public static class ShooterConstants {
		public static final boolean enabled = false;
		public static final boolean debug = false;

		public static final int flywheelMotor1Id = 30;
		public static final double flywheel1P = 0.1;
		public static final double flywheel1I = 0.2;
		public static final double flywheel1D = 0.3;
		public static final double flywheel1CurrentLimit = 40.0;
		public static final boolean invertFlywheelMotor1 = false;

		public static final int flywheelMotor2Id = 31;
		public static final double flywheel2P = 0.1;
		public static final double flywheel2I = 0.2;
		public static final double flywheel2D = 0.3;
		public static final double flywheel2CurrentLimit = 40.0;
		public static final boolean invertFlywheelMotor2 = false;

		public static final int turretMotorId = 32;
		public static final boolean invertTurretMotor = false;
		public static final double turretP = 50.0;
		public static final double turretI = 0.0;
		public static final double turretD = 0.0;
		public static final double minTurretPosition = 0.0;
		public static final double maxTurretPosition = 100.0;
		public static final double turretMotorMaxMotionCruiseVelocity = 10000;
		public static final double turretMotorMaxMotionMaxAcceleration = 10000;
		public static final double turretMotorMaxMotionAllowedProfileError = 20.0;
		public static final int turretMotorCurrentLimit = 10;

		public static final int indexerMotorId = 33;
		public static final boolean invertIndexerMotor = false;
		public static final double indexMotorSpeed = 12.0;
		public static final int indexerMotorCurrentLimit = 10;

		public static final int kickMotor1Id = 34;
		public static final boolean invertKickMotor1 = false;
		public static final double kickMotor1Speed = 12.0;

		public static final int kickMotor2Id = 35;
		public static final boolean invertKickMotor2 = false;
		public static final double kickMotor2Speed = 12.0;

		public static final int kickMotorCurrentLimit = 10;

		// public static final int hoodMotorId = 32;
		// public static final boolean invertHoodMotor = false;
		// public static final double hoodP = 0.1;
		// public static final double hoodI = 0.2;
		// public static final double hoodD = 0.3;

		public static final int hoodActuator1Port = 0;
		public static final int hoodActuator2Port = 1;
	}

}
