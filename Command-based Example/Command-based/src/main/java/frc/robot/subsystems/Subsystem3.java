package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ExampleObject;

public class Subsystem3 extends SubsystemBase{
    private ExampleObject object1;
    private ExampleObject object2;
    private ExampleObject object3;
    private Subsystem1 subsystem1;
    public Subsystem3(int num, Subsystem1 subsystem1){
        object1 = new ExampleObject(num);
        object2 = Constants.Subsystem3.objectConstant;
        object3 = new ExampleObject(num / Constants.Subsystem3.num);
        this.subsystem1 = subsystem1;

        System.out.println("Subsystem" + num + " Initialized");
    }

    public void periodic(){
        //System.out.println(object1.getNum() / object2.divNum(0.5));
        if(object2.getNum() == 15.0){
            System.out.println(object3);
        }
    }
}
