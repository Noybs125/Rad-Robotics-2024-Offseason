// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;



public class RobotContainer {
  public final DriveTrain driveTrain;
 
  public CommandXboxController commBox = new CommandXboxController(2);

  public RobotContainer() {

    driveTrain = new DriveTrain();
    
    configureBindings();
  }

  private void configureBindings() {
    commBox.a().onTrue(driveTrain.runMotorsCommand(0.1))
        .onFalse(driveTrain.runMotorsCommand(0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
