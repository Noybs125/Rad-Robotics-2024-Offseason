package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanisms extends SubsystemBase {
    
    private CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);
    private CANSparkMax wrist =  new CANSparkMax(10, MotorType.kBrushless);
    private DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(0);
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
    private PIDController intakePID = new PIDController(0.1, 0, 0);
    private PIDController wristPID  = new PIDController(0.1, 0, 0);

    private double intakeSet = 0;
    private double wristSet = 0;

    public Mechanisms(){

    }
    
    public void periodic(){
        intake.set(intakePID.calculate(intakeEncoder.get(), intakeSet));
        wrist.set(wristPID.calculate(wristEncoder.get(), wristSet));
    }

    public void setIntakeAndWristPosition(double intakeSetpoint, double wristSetpoint){
        setIntakePositon(intakeSetpoint);
        setWristPosition(wristSetpoint);
    }

    public void setIntakePositon(double setpoint){
        intakeSet = setpoint;
    }

    public void setWristPosition(double setpoint){
        wristSet = setpoint;
    }
}
