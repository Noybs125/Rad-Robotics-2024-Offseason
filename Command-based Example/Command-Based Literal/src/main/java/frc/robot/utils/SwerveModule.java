package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModule {
    
    private CANSparkMax driveMotor;
    private CANSparkMax rotationMotor;
    private CANcoder moduleEncoder;

    private PIDController rotationPID = new PIDController(0.1, 0, 0);

    private Translation2d translation;
    private double encoderOffset;

    public SwerveModule(Translation2d translation, double encoderOffset, int driveMotorId, int rotationMotorId, int encoderId){
        this.translation = translation;
        this.encoderOffset = encoderOffset;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);
        moduleEncoder = new CANcoder(encoderId);
    }

    public void setPosition(double speed, double rotation){
        double encoderPosition = moduleEncoder.getAbsolutePosition().getValueAsDouble();

        driveMotor.set(speed);
        rotationMotor.set(
            rotationPID.calculate(encoderPosition, rotation - encoderOffset));
    }
}
