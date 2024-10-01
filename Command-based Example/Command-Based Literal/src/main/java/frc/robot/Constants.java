package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
    public static class Controls {

    }
    
    public static class DrivetrainConstants{

    }
    public static class VisionConstants {
        public static final Transform3d aprilTagCamFrontTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Transform3d aprilTagCamBackTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Transform3d gamePieceCamTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
        
    }
    public static class MechanismsConstants {

    }
}
