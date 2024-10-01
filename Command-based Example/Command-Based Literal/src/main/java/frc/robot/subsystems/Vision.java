package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.Camera;

public class Vision extends SubsystemBase {

    public Camera aprilTagCamFront;
    public Camera aprilTagCamBack;
    public Camera gamePieceCam;
    private ArrayList<Camera> camList = new ArrayList<>();

    private PIDController rotatePID = new PIDController(0.1, 0, 0);
    
    public Vision(){
        aprilTagCamFront = new Camera("aprilTagFront", VisionConstants.aprilTagCamFrontTransform);
        aprilTagCamBack = new Camera("aprilTagBack", VisionConstants.aprilTagCamBackTransform);
        gamePieceCam = new Camera("gamePiece", VisionConstants.gamePieceCamTransform);
        camList.add(aprilTagCamFront);
        camList.add(aprilTagCamBack);
        camList.add(gamePieceCam);
    }

    public void periodic(){
        for (Camera camera : camList) {
            camera.result = camera.getLatestResult();
        }
    }

    public double getTargetRotation(Camera cam){
        double rotation = rotatePID.calculate(cam.getLatestResult().getBestTarget().getYaw(), 0);
        return rotation;
    }

    public double getWristSetpoint(){
        double scoreHeight = Units.inchesToMeters(84);
        double scoreDistance = aprilTagCamFront.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
        double angle = Math.toDegrees(Math.atan(scoreHeight / scoreDistance));
        return angle;
    }
}
