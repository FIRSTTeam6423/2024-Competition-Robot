// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Collections;
import java.util.List;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionUtil extends SubsystemBase {

    private final PhotonCamera colorCam = new PhotonCamera("colorcam");
	private final PhotonCamera aprilCamFront = new PhotonCamera("aprilcamfront");
	private final PhotonCamera aprilCamBack = new PhotonCamera("aprilcamback");
    public double allianceOrientation = 0;

    // Gets April Tag coords of a specified id
    public Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}

	// Get's robots position based on the nearest april tag
    public List<Pose2d> getVisionRobotPoseMeters() {
		// Gets April tag targets from back and front cameras
        PhotonPipelineResult backResults = aprilCamFront.getLatestResult();
		PhotonPipelineResult frontResults = aprilCamBack.getLatestResult();
        PhotonTrackedTarget FrontTarget = backResults.getBestTarget();
		PhotonTrackedTarget BackTarget = frontResults.getBestTarget();

		// Proccesses front results
		Pose2d frontEstimates;
		if (frontResults.hasTargets() == true) {
			frontEstimates = PhotonUtils.estimateFieldToRobotAprilTag(
						FrontTarget.getBestCameraToTarget(), 
						getTagPose3dFromId(FrontTarget.getFiducialId()), 
						Constants.CAMERA_TO_ROBOT
					).toPose2d();
		} else {
			frontEstimates = null;
		}
		
		// Proccesses back results
		Pose2d backEstimates;
		if (backResults.hasTargets() == true) {
			backEstimates = PhotonUtils.estimateFieldToRobotAprilTag(
						BackTarget.getBestCameraToTarget(), 
						getTagPose3dFromId(BackTarget.getFiducialId()), 
						Constants.CAMERA_TO_ROBOT
					).toPose2d();
		} else {
			backEstimates = null;
		}
		
		// Returns position estimates
		List<Pose2d> aprilPairs = new ArrayList<>(2);
		aprilPairs.add(frontEstimates);
		aprilPairs.add(backEstimates);
		return aprilPairs;
    }
}