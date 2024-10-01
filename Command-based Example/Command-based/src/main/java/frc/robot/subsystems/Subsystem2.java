package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsystem2 extends SubsystemBase{
    private boolean trueFalse;
    public Subsystem2(){
        trueFalse = false;

        System.out.println("Subsystem2 Initialized");
    }

    public void periodic(){
        if(trueFalse){
            trueFalse = false;
        }else{
            trueFalse = true;
        }
    }

    public boolean trueFalse(){
        return trueFalse;
    }
}
