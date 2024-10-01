// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SuperStructure.RobotState;
import frc.robot.subsystems.mechanisms.Flywheels;
import frc.robot.subsystems.mechanisms.Mechanisms;

public class RobotContainer {
  public final XboxController xbox = new XboxController(0);
  public final CommandXboxController commXbox = new CommandXboxController(0);

  public final Drivetrain driveTrain;
  public final Vision vision;
  public final Flywheels flywheels;
  public final Mechanisms mechanisms;
  public final Sensors sensors;
  public final Autonomous autonomous;
  public final SuperStructure superStructure;

  public RobotContainer() {
    driveTrain = new Drivetrain();
    vision = new Vision();
    flywheels = new Flywheels();
    mechanisms = new Mechanisms();
    sensors = new Sensors();
    autonomous = new Autonomous();
    superStructure = new SuperStructure(flywheels, mechanisms, autonomous, driveTrain, sensors, vision, xbox);

    configureBindings();
  }

  private void configureBindings() {

    Trigger driveHalfSpeedButton = commXbox.rightBumper();
    Trigger groundIntakeButton = commXbox.a();
    Trigger getGamePieceButton = commXbox.b();

    driveTrain.setDefaultCommand(driveTrain.driveCommand(
        () -> superStructure.swerveYTranslation, 
        () -> superStructure.swerveXTranslation, 
        () -> superStructure.swerveRotation, 
        () -> superStructure.swerveIsFieldRelative
        ));


    driveHalfSpeedButton.whileTrue(driveTrain.driveCommand(
        () -> superStructure.swerveYTranslation / 2, 
        () -> superStructure.swerveXTranslation / 2, 
        () -> superStructure.swerveRotation     / 2, 
        () -> superStructure.swerveIsFieldRelative
        ));

    groundIntakeButton.onTrue(superStructure.setRobotStateCommand(RobotState.GroundIntake));

    getGamePieceButton.onTrue(superStructure.setRobotStateCommand(RobotState.GetGamePiece));

    groundIntakeButton.negate()
        .and(getGamePieceButton.negate())
        .onTrue(superStructure.setRobotStateCommand(RobotState.Default));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
