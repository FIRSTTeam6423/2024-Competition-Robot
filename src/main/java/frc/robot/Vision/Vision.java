// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commons.VisionUpdate;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

public class Vision extends SubsystemBase {
	
	// Initalizing the cameras
	private final PhotonCamera aprilCam;
	private final PhotonPoseEstimator estimator;
    public double allianceOrientation = 0;
	
	public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}
	
	public static boolean inField(Pose3d pose) {
      return (pose.getX() > 0
          && pose.getX() < VisionConstants.LENGTH.in(Meters)
          && pose.getY() > 0
          && pose.getY() < VisionConstants.WIDTH.in(Meters)
		);
    }

    // Gets April Tag coords of a specified id
    public Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}

	public Vision() {
		aprilCam = new PhotonCamera("aprilcam");
		estimator = new PhotonPoseEstimator(
			VisionConstants.TagPoses,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // * This will try and use multiple tags to determine position
			aprilCam,
			VisionConstants.APRIL_CAMERA_TO_ROBOT
		);
		
		estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // * If there aren't multiple tags or the multi tag calculations fail, it will just go for the tag that it's most sure of
	}

	public EstimatedRobotPose getVisionPoseUpdatesMeterss() {
		//List<PoseEstimate> estimates = new ArrayList<>();
		PhotonPipelineResult result = aprilCam.getLatestResult();
		var estimate = estimator.update(result);
		// estimate
        //   .filter(
        //       f ->x`
        //           inField(f.estimatedPose)
        //               && Math.abs(f.estimatedPose.getZ()) < VisionConstants.CAM_HEIGHT
        //               && Math.abs(f.estimatedPose.getRotation().getX()) < VisionConstants.MAX_ANGLE
        //               && Math.abs(f.estimatedPose.getRotation().getY()) < VisionConstants.MAX_ANGLE)
        //   .ifPresent(
        //       e ->
        //           estimates.add(
        //               new PoseEstimate(
        //                   e, getEstimationStdDevs(e.estimatedPose.toPose2d(), result))));

		return estimate.get();
	}

	// Gets robot's position based on the nearest April Tags
	// Now we return a list of VisionUpdates
    public List<VisionUpdate> getVisionPoseUpdatesMeters() {

		// Gets front and back updates
		List<VisionUpdate> updates = new ArrayList<>();

		// Gets April Tag targets from back and front cameras
		PhotonPipelineResult results = aprilCam.getLatestResult();
        List<PhotonTrackedTarget> targets = new ArrayList<>();
		targets.add(results.getBestTarget());

		// Get April Tag estimates
		for (PhotonTrackedTarget target: targets) {
			Pose2d estimate = PhotonUtils.estimateFieldToRobotAprilTag(
						target.getBestCameraToTarget(), 
						getTagPose3dFromId(target.getFiducialId()),
						VisionConstants.APRIL_CAMERA_TO_ROBOT
				).toPose2d();
			updates.add(new VisionUpdate(estimate, results.getTimestampSeconds()));
			
			if (estimate == null) System.err.println("ESTIMATE IS NULL: ");
		}

		// Return list of estimates
		return updates;
    }

	// public Matrix<N3, N1> getEstimationStdDevs(
    //   Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
	// 	var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
	// 	var targets = pipelineResult.getTargets();
	// 	int numTags = 0;
	// 	double avgDist = 0;
	// 	double avgWeight = 0;
	// 	for (var tgt : targets) {
	// 	var tagPose = VisionConstants.TagPoses.getTagPose(tgt.getFiducialId());
	// 	if (tagPose.isEmpty()) continue;
	// 	numTags++;
	// 	avgDist +=
	// 		tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
	// 	avgWeight += VisionConstants.TAG_WEIGHTS[tgt.getFiducialId() - 1];
	// 	}
	// 	if (numTags == 0) return estStdDevs;

	// 	avgDist /= numTags;
	// 	avgWeight /= numTags;

	// 	// Decrease std devs if multiple targets are visibleX
	// 	if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
	// 	// Increase std devs based on (average) distance
	// 	if (numTags == 1 && avgDist > 4)
	// 	estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
	// 	else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

	// 	estStdDevs = estStdDevs.times(avgWeight);

	// 	return estStdDevs;
    // }
}