package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ExampleObject;

public class Subsystem1 extends SubsystemBase { // extended to use various methods
    // declare variables here
    private double var1;
    private String var2;
    private ExampleObject var3;

    private int count = 0;

    public Subsystem1(double var1, String var2, ExampleObject var3){ // this is run once when a new object of this class is created
        // initialize variables/other code here
        this.var1 = var1; // "this" prefix is used to disambiguate between the object variable and the constructor variable
        this.var2 = var2;
        this.var3 = var3;

        System.out.println("Subsystem1 initialized");
    }

    public void periodic(){ // runs periodically. called by SubsystemBase
        if(var1 == Math.PI){
            count++;
            System.out.println(count);
        }
    }

    public void exampleMethod(){
        System.out.println("exampleString");
    }

    public Command exampleCommand(){
        return new InstantCommand(() -> exampleMethod()); // MUST be a lambda
    }
}
