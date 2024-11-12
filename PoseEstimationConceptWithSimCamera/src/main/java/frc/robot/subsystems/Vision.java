package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.*;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.Constants.vision;

public class Vision extends SubsystemBase{

    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private PhotonPipelineResult latestResult;
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget;
    private int bestTargetId;
    private VisionSystemSim visionSim = new VisionSystemSim("main");
    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);    
    private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, bestCameraToTarget);
    private EstimatedRobotPose estRobotPose = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);

    public Vision(AHRS gyro){
        this.gyro = gyro;
    }
    public void simulationInit(){
        visionSim.addAprilTags(aprilTagFieldLayout);
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
    } 

    public void periodic(){
        latestResult = camera.getLatestResult();
        
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        if(getEstimatedGlobalPose(robotPose).isPresent()){
            estRobotPose = getEstimatedGlobalPose(robotPose).get();
            robotPose = estRobotPose.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        }
    }

    public boolean updatePose(){
        if(apriltagMap.getTagPose(bestTargetId).isPresent()){
            return true;
        }
        return false;
    }
    public Pose2d getRobotPose(){
        robotPose = estRobotPose.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        return robotPose;
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
