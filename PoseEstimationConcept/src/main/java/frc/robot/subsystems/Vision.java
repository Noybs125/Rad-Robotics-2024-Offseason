package frc.robot.subsystems;

import java.io.IOException;

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
import frc.robot.Constants;

public class Vision extends SubsystemBase{

    private final PhotonCamera limelight = new PhotonCamera("photonvision");
    private PhotonPipelineResult latestResult;
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget;
    private int bestTargetId;

    private final AHRS gyro;
    private final Swerve swerve;

    public Pose2d robotPose = new Pose2d();
    private Field2d field = new Field2d();

    private ComplexWidget fieldPublisher;

    private AprilTagFieldLayout apriltagMap;
    {
        try {
            apriltagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }

    public Vision(AHRS gyro, Swerve swerve){
        this.gyro = gyro;
        this.swerve = swerve;

        fieldPublisher = Shuffleboard.getTab("Odometry").add("field odometry", field).withWidget("Field");
    }

    public void periodic(){
        latestResult = limelight.getLatestResult();
        
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
            
            calculateRobotPose();
        }
        
        field.setRobotPose(swerve.getPose());
    }

    private void calculateRobotPose(){
        if(apriltagMap.getTagPose(bestTargetId).isPresent()){
            Pose3d tagLocation = apriltagMap.getTagPose(bestTargetId).get();
            Pose3d cameraPose = tagLocation.transformBy(bestCameraToTarget);
            robotPose = cameraPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
            swerve.resetOdometry(robotPose);
            //swerve.resetOdometry(new Pose2d(robotPose.getTranslation(), gyro.getRotation2d()));
        }
    }
}
