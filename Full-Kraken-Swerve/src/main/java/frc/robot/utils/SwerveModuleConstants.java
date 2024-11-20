package frc.robot.utils;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int canCoderID;
    public final double canCoderOffsetDegrees;
    public final boolean isInverted;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double canCoderOffsetDegrees, boolean isInverted) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.canCoderID = canCoderID;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
        this.isInverted = isInverted;
    }
}
