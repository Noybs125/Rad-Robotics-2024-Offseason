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
import frc.robot.Constants.vision;
import frc.robot.utils.Camera;

public class Vision extends SubsystemBase{

    private PhotonCamera camera1 = new PhotonCamera("photonvision1");
    private PhotonCamera camera2 = new PhotonCamera("photonvision2");
    private PhotonCamera camera3 = new PhotonCamera("photonvision3");



    private final AHRS gyro;

    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public EstimatedRobotPose estRobotPose1 = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);
    public EstimatedRobotPose estRobotPose2 = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);
    public EstimatedRobotPose estRobotPose3 = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);

    
    public final Camera limelight = new Camera(camera1, this, estRobotPose1);
    public final Camera orangepi1 = new Camera(camera2, this, estRobotPose2);
    public final Camera orangepi2 = new Camera(camera3, this, estRobotPose3);

    public Vision(AHRS gyro){
        this.gyro = gyro;
    }

    public void periodic(){
        robotPose = estRobotPose1.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        if(limelight.getEstimatedGlobalPose(robotPose).isPresent()){
            estRobotPose1 = limelight.getEstimatedGlobalPose(robotPose).get();
        }
        robotPose = estRobotPose2.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        if(orangepi1.getEstimatedGlobalPose(robotPose).isPresent()){
            estRobotPose2 = orangepi1.getEstimatedGlobalPose(robotPose).get();
        }
        robotPose = estRobotPose3.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        if(orangepi2.getEstimatedGlobalPose(robotPose).isPresent()){
            estRobotPose3 = orangepi2.getEstimatedGlobalPose(robotPose).get();
        }
        limelight.periodic();
        orangepi1.periodic();
        orangepi2.periodic();
    }
}
