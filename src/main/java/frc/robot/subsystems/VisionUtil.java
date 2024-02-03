// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commons.VisionUpdate;

import java.util.List;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionUtil extends SubsystemBase {
	
	// Initalizing the cameras
	private static final PhotonCamera aprilCamFront = new PhotonCamera("aprilcamfront");
	private static final PhotonCamera aprilCamBack = new PhotonCamera("aprilcamback");
    public double allianceOrientation = 0;

    // Gets April Tag coords of a specified id
    public Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}

	// Get's robots position based on the nearest april tag
	// now we return a list of 
    public List<VisionUpdate> getVisionPoseUpdatesMeters() {
		// Gets April tag targets from back and front cameras
        PhotonPipelineResult backResults = aprilCamFront.getLatestResult();
		PhotonPipelineResult frontResults = aprilCamBack.getLatestResult();
        List<PhotonTrackedTarget> frontTargets = backResults.getTargets();
		List<PhotonTrackedTarget> backTargets = frontResults.getTargets();

		// Returns position estimates
		List<VisionUpdate> updates = new ArrayList<>(2);

		// Proccesses front results
		Pose2d frontEstimates = null;
		for (PhotonTrackedTarget frontTarget: frontTargets) {
			if (frontResults.hasTargets() == true) {
				frontEstimates = PhotonUtils.estimateFieldToRobotAprilTag(
							frontTarget.getBestCameraToTarget(), 
							getTagPose3dFromId(frontTarget.getFiducialId()), 
							Constants.CAMERA_TO_ROBOT
						).toPose2d();
				updates.add(new VisionUpdate(frontEstimates, frontResults.getTimestampSeconds()));
			}
		}
		
		// Proccesses back results
		Pose2d backEstimates = null;
		for (PhotonTrackedTarget backTarget: backTargets) {
			if (backResults.hasTargets() == true) {
				backEstimates = PhotonUtils.estimateFieldToRobotAprilTag(
							backTarget.getBestCameraToTarget(), 
							getTagPose3dFromId(backTarget.getFiducialId()), 
							Constants.CAMERA_TO_ROBOT
						).toPose2d();
				updates.add(new VisionUpdate(backEstimates, backResults.getTimestampSeconds()));
			}
		}

		// Returns the averaged pose values
		return updates;
    }
}