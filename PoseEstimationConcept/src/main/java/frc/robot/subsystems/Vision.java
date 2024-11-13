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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;
import frc.robot.Constants;

public class Vision extends SubsystemBase{

    private final PhotonCamera limelight = new PhotonCamera("photonvision");
    private PhotonPipelineResult latestResult;
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget;
    private int bestTargetId;

    private final AHRS gyro;

    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, limelight, bestCameraToTarget);
    private EstimatedRobotPose estRobotPose = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);

    private AprilTagFieldLayout apriltagMap;
    {
        try {
            apriltagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }

    public Vision(AHRS gyro){
        this.gyro = gyro;
    }

    public void periodic(){
        latestResult = limelight.getLatestResult();
        if(photonPoseEstimator.getRobotToCameraTransform() != Constants.vision.cameraToRobotCenter){
                photonPoseEstimator.setRobotToCameraTransform(Constants.vision.cameraToRobotCenter);
        }
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        Optional<EstimatedRobotPose> estPose = getEstimatedGlobalPose(robotPose);
        if(estPose.isPresent()){
            estRobotPose = estPose.get();
            robotPose = estRobotPose.estimatedPose.toPose2d();
        }
    }

    public boolean updatePose(){
        if(apriltagMap.getTagPose(bestTargetId).isPresent()){
            return true;
        }
        return false;
    }
    public Pose2d getRobotPose(){
        robotPose = estRobotPose.estimatedPose.toPose2d();
        return robotPose;
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
