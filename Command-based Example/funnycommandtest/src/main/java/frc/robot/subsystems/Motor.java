package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase{
    private CANSparkMax motor = new CANSparkMax(12, MotorType.kBrushless);
    public Motor(){

    }

    private void setMotor(double value){
        motor.set(value);
    }

    public Command setMotorCommand(double value){
        return new InstantCommand(() -> setMotor(value));
    }
}
