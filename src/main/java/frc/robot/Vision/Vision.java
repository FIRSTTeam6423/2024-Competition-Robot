// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import frc.robot.Constants;
import frc.robot.commons.VisionUpdate;

import frc.robot.Vision.VisionConstants;

import java.util.List;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	
	// Initalizing the cameras
	private final PhotonCamera aprilCam = new PhotonCamera("aprilcam");
    public double allianceOrientation = 0;

    // Gets April Tag coords of a specified id
    public Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
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
}