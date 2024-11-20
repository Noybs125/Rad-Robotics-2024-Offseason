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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.Constants.vision;
import frc.robot.subsystems.Vision;

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
        this.photonPoseEstimator = new PhotonPoseEstimator(apriltagMap, PoseStrategy.AVERAGE_BEST_TARGETS, camera, bestCameraToTarget);

    }
    public void periodic(){
        latestResult = camera.getLatestResult();
        
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        if(getEstimatedGlobalPose(vision.robotPose).isPresent()){
            if(photonPoseEstimator.getRobotToCameraTransform() != Constants.vision.cameraToRobotCenter){
                photonPoseEstimator.setRobotToCameraTransform(Constants.vision.cameraToRobotCenter);
            }
            estRobotPose = getEstimatedGlobalPose(vision.robotPose).get();
        }
    }

    public boolean updatePose(){
        if(apriltagMap.getTagPose(bestTargetId).isPresent()){
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
}

