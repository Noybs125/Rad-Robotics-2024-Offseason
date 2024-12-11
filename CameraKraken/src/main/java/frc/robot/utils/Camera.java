package frc.robot.utils;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.Matrix;

public class Camera {
    private Vision vision;
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult latestResult;
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget;
    private EstimatedRobotPose estRobotPose;
    public int bestTargetId;
    private AprilTagFieldLayout apriltagMap;
    private Pose2d newPose = new Pose2d();
    {
        try {
            apriltagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }

    AHRS gyro;

   
    public Camera(PhotonCamera camera, Vision vision, EstimatedRobotPose estRobotPose) {
        this.camera = camera;
        this.vision = vision;
        this.estRobotPose = estRobotPose;
        this.photonPoseEstimator = new PhotonPoseEstimator(apriltagMap, PoseStrategy.AVERAGE_BEST_TARGETS, camera, Constants.vision.cameraToRobotCenter);

    }
    public void periodic(){
        latestResult = camera.getLatestResult();
        
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        Optional<EstimatedRobotPose> estPose = getEstimatedGlobalPose(vision.robotPose);
        if(estPose.isPresent()){
            if(photonPoseEstimator.getRobotToCameraTransform() != Constants.vision.cameraToRobotCenter){
                photonPoseEstimator.setRobotToCameraTransform(Constants.vision.cameraToRobotCenter);
            }
            estRobotPose = estPose.get();
        }
    }

    public boolean updatePose(){
        if(camera.getLatestResult().hasTargets()){
            return true;
        }
        return false;
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
    public Pose2d getRobotPose(){
        newPose = estRobotPose.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        return newPose;
    }
    public Matrix<N3, N1> getPoseAmbiguity(){
        double smallestDistance = Double.POSITIVE_INFINITY;
            if(estRobotPose != null){
            for (var target : estRobotPose.targetsUsed) {
                var t3d = target.getBestCameraToTarget();
                var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
                if (distance < smallestDistance) {
                smallestDistance = distance;
                }
            }
            double poseAmbiguityFactor = estRobotPose.targetsUsed.size() != 1
                ? 1
                : Math.max(1, estRobotPose.targetsUsed.get(0).getPoseAmbiguity() + Constants.vision.POSE_AMBIGUITY_SHIFTER * Constants.vision.POSE_AMBIGUITY_MULTIPLIER);
            double confidenceMultiplier = Math.max(1,(Math.max(1, Math.max(0, smallestDistance - Constants.vision.NOISY_DISTANCE_METERS) * Constants.vision.DISTANCE_WEIGHT) * poseAmbiguityFactor) / (1 + ((estRobotPose.targetsUsed.size() - 1) * Constants.vision.TAG_PRESENCE_WEIGHT)));
            SmartDashboard.putNumber(camera.getName(), confidenceMultiplier);
            return Constants.vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
        }
        return Constants.vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(0);
    }
}

