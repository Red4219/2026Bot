package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight {
	private RawDetection[] rawDetections;
	private RawFiducial[] rawFiducials;
	private int[] detections;
	private int counter = 0;
	private LimelightHelpers.PoseEstimate mt2;
	private Pose2d robotPose2d = null;

	private double[] poseArray = new double[3];
	private String name = "";

	private int targetId = 0;

    public Limelight(String name) {
		LimelightHelpers.setCameraPose_RobotSpace(

			name, 
			0, 
			0, 
			0, 
			0, 
			0, 
			180

		);
		this.name = name;
	}

	/*public PoseEstimate getPose2d(Pose2d robotPose2d) {

		LimelightHelpers.SetRobotOrientation(
									name,
									robotPose2d.getRotation().getDegrees(),
									0,
									0,
									0,
									0,
									0);


		mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

		return mt2;
	}*/

	public boolean lookForTarget(int target) {
		
		int[] detections = getDetections();

		for(int i = 0; i < detections.length; i++) {
			if(detections[i] == target) {
				return true;
			}
		}

		return false;
	}

	public boolean hasTarget() {
		return LimelightHelpers.getTV(Constants.LimelightConstants.name1);
	}

	public void setPipelineIndex(int index) {
		LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.name1, index);
	}

	public double getDistancToTargetFromRobot(int target) {
		rawFiducials = LimelightHelpers.getRawFiducials(Constants.LimelightConstants.name1);

		return rawFiducials[target].distToRobot;
	}

	public double getHorizontalOffsetFromTarget(int target) {
		rawFiducials = LimelightHelpers.getRawFiducials(Constants.LimelightConstants.name1);

		return rawFiducials[target].txnc;
	}

	public int[] getDetections() {
		rawDetections = LimelightHelpers.getRawDetections(Constants.LimelightConstants.name1);

		counter = 0;
		detections = new int[rawDetections.length];
		for (RawDetection detection : rawDetections) {
			detections[counter] = detection.classId;
			counter++;
		}

		return detections;
	}

	public void setTargetId(int targetId) {
		this.targetId = targetId;
	}

	public RawDetection targetDetection() {
		rawDetections = LimelightHelpers.getRawDetections(Constants.LimelightConstants.name1);

		for (RawDetection detection : rawDetections) {
			if( detection.classId == this.targetId) {
				return detection;
			}
		}

		return null;
	}
}
