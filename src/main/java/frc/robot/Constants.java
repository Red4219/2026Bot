// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(18.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }


  public static class OperatorConstants {

    public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PhotonVisionConstants {

    public static boolean kEnablePhotonVision = true;
	  public static boolean kDebugPhotonVision = true;
    public static boolean kEnablePhotonVisionCamera1 = true;
	  public static boolean kEnablePhotonVisionCamera2 = true;

		public static final boolean debugPhotonVision = true;

		//public static final PoseStrategy poseStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
		public static final PoseStrategy poseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

		public static double camDiagFOV = 170.0;
		public static double camPitch = 0.0;
		public static double cam2Pitch = 0.0;
		public static double camHeightOffGround = Units.inchesToMeters(40.0);
		public static double cam2HeightOffGround = Units.inchesToMeters(40.0);
		// the side to side position of the camera relative to the robot center
		public static double camX = Units.inchesToMeters(0);
		public static double cam2X = Units.inchesToMeters(0);
		// the front to back position of the camera relative to the robot center
		public static double camY = Units.inchesToMeters(-7.5);
		public static double cam2Y = Units.inchesToMeters(-7.5);

		public static Transform3d cameraToRobot = new Transform3d(
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
		public static Transform3d camera2ToRobot = new Transform3d(
                    new Translation3d(
						cam2X,
						cam2Y,
					 	PhotonVisionConstants.cam2HeightOffGround
					),
					new Rotation3d(
						0,
						PhotonVisionConstants.cam2Pitch,
						Math.toRadians(180)
					)
				);

		public static final String CameraName = "cam1";
		public static final String Camera2Name = "cam2";
		//public static final boolean kEnableCamera1 = true;
		//public static final boolean kEnableCamera2 = true;


		// Simulated Vision System.
    	// Configure these to match your PhotonVision Camera,
    	// pipeline, and LED setup.
		public static double sim_camDiagFOV = camDiagFOV; // degrees - assume wide-angle camera
		public static double sim_camPitch = camPitch; // degrees
    	public static double sim_camHeightOffGround = camHeightOffGround; // meters
    	//public static double sim_maxLEDRange = 20; // meters
    	public static int sim_camResolutionWidth = 640; // pixels
    	public static int sim_camResolutionHeight = 480; // pixels
    	//public static double sim_minTargetArea = 10; // square pixels
		//public static double sim_minTargetArea = 300; // square pixels

		/**
    	* Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    	* less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    	*/
		public static double visionMeasurementStdDevsX = 0.5;
		public static double visionMeasurementStdDevsY = 0.5;
		public static double visionMeasurementStdDevsTheta = Units.degreesToRadians(10);

		// The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
	}
}