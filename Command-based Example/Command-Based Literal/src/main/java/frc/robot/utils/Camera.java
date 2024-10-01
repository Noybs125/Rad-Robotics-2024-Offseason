package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera extends PhotonCamera {

    private final Transform3d cameraOffset;
    public PhotonPipelineResult result;

    public Camera(String name, Transform3d cameraOffset){
        super(name);
        this.cameraOffset = cameraOffset;
    }

    public Transform3d getCameraOffset(){
        return cameraOffset;
    }

    public Pose2d getRobotPose(){
        result = super.getLatestResult();
        Pose3d pose = new Pose3d();
        // determine pose here
        return pose.toPose2d();
    }

    public Pose2d getGamePiecePose(){
        result = super.getLatestResult();
        Pose2d pose = new Pose2d();
        // determine gamepiece pose here
        return pose;
    }

}
