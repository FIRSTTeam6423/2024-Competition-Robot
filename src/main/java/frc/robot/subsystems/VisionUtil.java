// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Collections;
import java.util.List;

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

    private final PhotonCamera colorcam = new PhotonCamera("colorcam");
	private final PhotonCamera aprilcamfront = new PhotonCamera("aprilcamfront");
	private final PhotonCamera aprilcamback = new PhotonCamera("aprilcamback");
    public double allianceOrientation = 0;

    // Gets April Tag coords of a specified id
    public Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}
	
	// ? I don't think this is necessary lol?
    /*
	public PhotonTrackedTarget getNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget();
		}
		return null;
	} */

    //
	public List<PhotonTrackedTarget> getAllCameraTargets() {
		PhotonPipelineResult result = colorcam.getLatestResult();
		if (result.hasTargets()) {
			return result.getTargets();
		} else {
			return Collections.<PhotonTrackedTarget>emptyList();
		}
	}

    //
	public Pose3d getPose3dOfNearestCameraTarget() {
		PhotonPipelineResult result = colorcam.getLatestResult();
		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
			Pose3d tagPose = getTagPose3dFromId(target.getFiducialId());
			return tagPose;
		}
        DriverStation.reportWarning("No nearest camera target to get Pose3d!", false);
		return null;
	}

    //
	public Pose2d getFieldPosed2dFromNearestCameraTarget() {
		PhotonPipelineResult result = colorcam.getLatestResult();
		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
			Pose3d tagPose = getTagPose3dFromId(target.getFiducialId());
			Pose3d pos = PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),   
          tagPose,
          Constants.CAMERA_TO_ROBOT // TODO: ADD THIS
		);
        allianceOrientation = Math.toDegrees(tagPose.getRotation().getZ());
			return new Pose2d(
					pos.getX(),
					pos.getY(),
					new Rotation2d(pos.getRotation().getZ()));
		}
		DriverStation.reportWarning("Could not get Pose2d from camera target: no targets found.", false);
		return null;
    }

    // Compute the robot's field-relative position exclusively from vision measurements.
    // Pose3d visionMeasurement3d =
    //     objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);

    // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
    // Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();

    public Pose2d getVisionRobotPoseUpdates() {
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html
		// https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-an-apriltagfieldlayout
        // Method to get the robot pose in meters using PhotonVision
        PhotonPipelineResult result = colorcam.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		if (result.hasTargets() == true) {
			return PhotonUtils.estimateFieldToRobotAprilTag(
						target.getBestCameraToTarget(), 
						getTagPose3dFromId(target.getFiducialId()), 
						Constants.CAMERA_TO_ROBOT
					).toPose2d();
		} else {
			return null;
		}
    }
}