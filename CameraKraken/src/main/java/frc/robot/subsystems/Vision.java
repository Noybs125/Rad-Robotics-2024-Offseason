package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.Constants;
import frc.robot.utils.Camera;

public class Vision extends SubsystemBase{
    // we on longer have a limelight on the robot, however we may one day need to put it back on again. Therefore, I have left this code inside of the program, although it may make it less readable, it could be useful one day. Thank you for taking the time to read this wonderful message and I hope you have a great day :D
    //private PhotonCamera camera1 = new PhotonCamera("photonvision1");
    private PhotonCamera camera2 = new PhotonCamera("OV9281_1");
    private PhotonCamera camera3 = new PhotonCamera("OV9281_2");



    private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();
    public EstimatedRobotPose estRobotPose1 = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);
    public EstimatedRobotPose estRobotPose2 = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);
    public EstimatedRobotPose estRobotPose3 = new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS);

    
    //public final Camera limelight = new Camera(camera1, this, estRobotPose1);
    public final Camera orangepi1 = new Camera(camera2, this, estRobotPose2);
    public final Camera orangepi2 = new Camera(camera3, this, estRobotPose3);

    public Vision(AHRS gyro){
        this.gyro = gyro;
    }

    public void periodic(){
        //robotPose = estRobotPose1.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        //if(limelight.getEstimatedGlobalPose(robotPose).isPresent())
        //{
        //    estRobotPose1 = limelight.getEstimatedGlobalPose(robotPose).get();
        //}
        Optional<EstimatedRobotPose> estPose = orangepi1.getEstimatedGlobalPose(robotPose);
        robotPose = estRobotPose2.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        if(estPose.isPresent())
        {
            estRobotPose2 = estPose.get();
        }
        estPose = orangepi2.getEstimatedGlobalPose(robotPose);
        robotPose = estRobotPose3.estimatedPose.transformBy(Constants.vision.cameraToRobotCenter).toPose2d();
        if(estPose.isPresent())
        {
            estRobotPose3 = estPose.get();
        }
        //limelight.periodic();
        orangepi1.periodic();
        orangepi2.periodic();
    }
}
