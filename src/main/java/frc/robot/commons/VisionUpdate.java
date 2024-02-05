package frc.robot.commons;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionUpdate {
    private Pose2d pose2d;
    private double timestamp;

    public VisionUpdate(Pose2d pose2d, double timestamp){
        this.pose2d = pose2d;
        this.timestamp = timestamp;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }

    public double getTimestamp() {
        return timestamp;
    }

}