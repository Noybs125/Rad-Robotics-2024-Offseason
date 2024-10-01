package frc.robot;

import frc.robot.subsystems.Subsystem2;
import frc.robot.utils.ExampleObject;

/** 
 * The Constants class contains classes with variables that never change.
 * It's intended use is to easily access and be able to manually tweak 
 * variables in one place.
 */
public class Constants {
    public static class Controls {
        public static final int xboxControllerPort = 0;
        public static final int joystickPort = 2;
    }
    public static class Subsystem1 {
        public static final double number = 0.0;
        public static final String string = "example string";
    }
    public static class Subsystem3 {
        public static final double num = Math.sqrt(Math.PI * 12);
        public static final ExampleObject objectConstant = new ExampleObject(7.5);
    }
}
