package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.Flywheels;
import frc.robot.subsystems.mechanisms.Mechanisms;

public class SuperStructure extends SubsystemBase{
    
    public enum RobotState {
        GroundIntake,
        GetGamePiece,
        HasGamepiece,
        Auto,
        Custom,
        Default,
    }

    private RobotState robotState;
    private XboxController xbox;

    private Flywheels flywheels;
    private Mechanisms mechanisms;
    private Autonomous autonomous;
    private Drivetrain drivetrain;
    private Sensors sensors;
    private Vision vision;

    public double swerveYTranslation;
    public double swerveXTranslation;
    public double swerveRotation;
    public boolean swerveIsFieldRelative;

    public SuperStructure(Flywheels flyWheels, Mechanisms mechanisms, Autonomous autonomous, Drivetrain drivetrain, Sensors sensors, Vision vision, XboxController xbox){
        this.autonomous = autonomous;
        this.drivetrain = drivetrain;
        this.flywheels = flyWheels;
        this.mechanisms = mechanisms;
        this.sensors = sensors;
        this.vision = vision;
        this.xbox = xbox;
    }

    private void setRobotState(RobotState state){
        switch(state){
            case GroundIntake:
                groundIntake();
                break;
            case GetGamePiece:
                getGamePiece();
                break;
            case HasGamepiece:
                hasGamepiece();
                break;
            case Auto:
                auto();
                break;
            case Custom:
                custom();
                break;
            case Default:
            default:
                defaultState();
                break;
        }
        robotState = state;
    }

    public Command setRobotStateCommand(RobotState state){
        return runOnce(() -> setRobotState(state));
    }

    public RobotState getRobotState(){
        return robotState;
    }
    /**
     * This method handles the drive variables for the swerve chassis
     */
    public void setSwerveVars(DoubleSupplier yTranslation, DoubleSupplier xTranslation, DoubleSupplier rotation, boolean isFieldRelative){ 
        if(yTranslation != null){
            swerveYTranslation = yTranslation.getAsDouble();
        }
        if(xTranslation != null){
            swerveXTranslation = xTranslation.getAsDouble();
        }
        if(rotation != null){
            swerveRotation = rotation.getAsDouble();
        }

        swerveIsFieldRelative = isFieldRelative;
    }

    private void groundIntake(){
        defaultState();
        mechanisms.setIntakeAndWristPosition(-33.4, 67.1); // random placeholder values
        flywheels.setAllWheels(-1, -1, -1);
        System.out.println("groundIntake");
    }
    private void getGamePiece(){
        defaultState();
        setSwerveVars(null, null, () -> vision.getTargetRotation(vision.gamePieceCam), false);
    }
    private void hasGamepiece(){
        mechanisms.setIntakeAndWristPosition(0, vision.getWristSetpoint());
        flywheels.setAllWheels(0, 1, 0);
        setSwerveVars(() -> xbox.getLeftY(), () -> xbox.getLeftX(), () -> vision.getTargetRotation(vision.aprilTagCamFront), true);
    }
    private void auto(){

    }
    private void custom(){

    }
    private void defaultState(){ // default is a reserved word
        mechanisms.setIntakeAndWristPosition(0, 0);
        flywheels.setAllWheels(0, 0, 0);
        setSwerveVars(() -> xbox.getLeftY(), () -> xbox.getLeftX(), () -> xbox.getRightX(), true);
        System.out.println("default");
    }
}

