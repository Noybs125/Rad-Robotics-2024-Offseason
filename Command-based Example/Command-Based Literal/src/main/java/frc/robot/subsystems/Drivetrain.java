package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SwerveModule;

public class Drivetrain extends SubsystemBase {

    private SwerveModule frontLeftModule = new SwerveModule(new Translation2d(-1, 1), 90, 0, 1, 8);
    private SwerveModule frontRightModule = new SwerveModule(new Translation2d(1, 1), 0, 2, 3, 9);
    private SwerveModule backLeftModule = new SwerveModule(new Translation2d(-1, -1), 180, 4, 5, 10);
    private SwerveModule backRightModule = new SwerveModule(new Translation2d(1, -1), 270, 6, 7, 11);
    private ArrayList<SwerveModule> swerveModuleList = new ArrayList<>();
    
    public Drivetrain(){
        swerveModuleList.add(frontLeftModule);
        swerveModuleList.add(frontRightModule);
        swerveModuleList.add(backLeftModule);
        swerveModuleList.add(backRightModule);
    }
    
    public void periodic(){

    }

    private void drive(double yTranslation, double xTranslation, double rotation, boolean isFieldRelative){
        double moduleSpeed = 0;
        double moduleRotation = 0;
        // math to determine the values of these variables goes here

        for (SwerveModule swerveModule : swerveModuleList) {
            swerveModule.setPosition(moduleSpeed, moduleRotation);
        }
    }

    public Command driveCommand(DoubleSupplier yTranslation, DoubleSupplier xTranslation, DoubleSupplier rotation, BooleanSupplier isFieldRelative){
        return run(() -> {
            double yTranslationDouble = yTranslation.getAsDouble();
            double xTranslationDouble = xTranslation.getAsDouble();
            double rotationDouble = rotation.getAsDouble();
            boolean isFieldRelativeDouble = isFieldRelative.getAsBoolean();

            drive(yTranslationDouble, xTranslationDouble, rotationDouble, isFieldRelativeDouble);
        });
    }
}
