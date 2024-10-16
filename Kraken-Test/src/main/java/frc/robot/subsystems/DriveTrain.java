package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveTrain {

    private TalonFX krakenSteer1 = new TalonFX(6);
    private TalonFX krakenSteer2 = new TalonFX(10);
    private TalonFX krakenSteer3 = new TalonFX(4);
    private TalonFX krakenSteer4 = new TalonFX(3);

    public Command runMotorsCommand(double speed) {
        return new InstantCommand(() -> runMotors(speed));
    }

    public void runMotors(double speed) {
        krakenSteer1.set(speed);
        krakenSteer2.set(speed);
        krakenSteer3.set(speed);
        krakenSteer4.set(speed);
    }

    
}
