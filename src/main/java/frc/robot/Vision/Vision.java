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
	private final PhotonCamera colorCam = new PhotonCamera("colorcam");
	private final PhotonCamera aprilCamFront = new PhotonCamera("aprilcamfront");
	private final PhotonCamera aprilCamBack = new PhotonCamera("aprilcamback");
    public double allianceOrientation = 0;

    // Gets April Tag coords of a specified id
    public Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}

	// Get camera April Tag positions
	public List<VisionUpdate> addAprilTagResults(boolean isFront) {
		List<VisionUpdate> updates = new ArrayList<>();

		// Gets April Tag targets from back and front cameras
		PhotonPipelineResult results;
		if (isFront) results = aprilCamFront.getLatestResult();
		else results = aprilCamBack.getLatestResult();
        List<PhotonTrackedTarget> targets = results.getTargets();

		// Register the camera to robot constants
		Transform3d camToRobot;
		if (isFront) camToRobot = VisionConstants.CAMERA_TO_ROBOT_2;
		else camToRobot = VisionConstants.CAMERA_TO_ROBOT_3;

		// Get April Tag estimates
		Pose2d estimates = null;
		for (PhotonTrackedTarget target: targets) {
			estimates = PhotonUtils.estimateFieldToRobotAprilTag(
						target.getBestCameraToTarget(), 
						getTagPose3dFromId(target.getFiducialId()),
						camToRobot
				).toPose2d();
			updates.add(new VisionUpdate(estimates, results.getTimestampSeconds()));
			
			if (estimates == null) {
				if (isFront) System.out.println("FRONT ESTIMATES IS NULL: ");
				else System.out.println("BACK ESTIMATES IS NULL: ");
			}
		}

		// Return list of estimates
		return updates;
	}

	// Gets robot's position based on the nearest April Tags
	// Now we return a list of VisionUpdates
    public List<VisionUpdate> getVisionPoseUpdatesMeters() {

		// Gets front and back updates
		List<VisionUpdate> frontUpdates = addAprilTagResults(true);
		List<VisionUpdate> backUpdates = addAprilTagResults(false);

		// Compiles front and back updates into a single list
		List<VisionUpdate> allUpdates = new ArrayList<>();
		allUpdates.addAll(frontUpdates);
		allUpdates.addAll(backUpdates);

		// Returns all the averaged pose values
		return allUpdates;
    }
}