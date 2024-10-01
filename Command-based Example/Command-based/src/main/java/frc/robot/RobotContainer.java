// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Subsystem1;
import frc.robot.subsystems.Subsystem2;
import frc.robot.subsystems.Subsystem3;
import frc.robot.utils.ExampleObject;


public class RobotContainer { // everything in this class is run once when this class is initialized
  // declare/initialize variables
  public final Subsystem1 subsystem1;
  public final Subsystem2 subsystem2;
  public final Subsystem3 subsystem3;

  public final XboxController xbox = new XboxController(Constants.Controls.xboxControllerPort);
  public final CommandXboxController commXbox = new CommandXboxController(Constants.Controls.xboxControllerPort);
  public RobotContainer() {
    // initialize variables. example: controllers, subsystems, triggers
    subsystem1 = new Subsystem1(1, Constants.Subsystem1.string, new ExampleObject(3.3));
    subsystem2 = new Subsystem2();
    subsystem3 = new Subsystem3(3, subsystem1);

    // set controller button/joystick bindings
    configureBindings();
  }
  /**
   * This method sets controller/other bindings using Triggers. A Trigger is an 
   * object in WPILib that, when initialized, repeatedly checks the 
   * condition in the parenthesis 
   */
  private void configureBindings() { 
    // define triggers

    Trigger exampleTrigger = commXbox.b(); // b() in the CommandXboxController object is object type Trigger
    Trigger exampleTrigger2 = new Trigger(() -> subsystem2.trueFalse()); // '() -> ' turns the following type into a supplier. in this case, 
    //                                                                      the boolean trueFalse() becomes a BooleanSupplier

    Trigger printButton = commXbox.a(); 
    Trigger rumbleButton = commXbox.rightTrigger(0.25);

    // set bindings to triggers
    exampleTrigger
        .onTrue(new InstantCommand(() -> System.out.println("example trigger is true")))
        .onFalse(new InstantCommand(() -> System.out.println("example trigger is false"))
        );

    printButton.onTrue(subsystem1.exampleCommand()); // the onTrue() method is run once when the trigger is changed from false to true

    rumbleButton
        .onTrue(                          
            new InstantCommand(                          // the InstantCommand object runs the action once
                () -> xbox.setRumble(RumbleType.kLeftRumble, 1)))
        .onFalse(
            new InstantCommand(
                () -> xbox.setRumble(RumbleType.kLeftRumble, 0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
