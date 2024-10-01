package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase {
    
    private CANSparkMax intakeWheels = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax shooterWheels = new CANSparkMax(12, MotorType.kBrushless);
    private VictorSPX indexWheels = new VictorSPX(13);

    public Flywheels(){

    }
    
    public void periodic(){

    }

    public void setAllWheels(double intake, double shooter, double index){
        intakeWheels.set(intake);
        shooterWheels.set(shooter);
        indexWheels.set(VictorSPXControlMode.PercentOutput, index);
    }
}
