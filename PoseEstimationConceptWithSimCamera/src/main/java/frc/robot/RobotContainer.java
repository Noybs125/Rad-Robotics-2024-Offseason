package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick leftJoy;
  public final Joystick rightJoy;
  public final JoystickButton rightJoyButtonTrig;
  public final XboxController xbox;
  public final CommandXboxController commXbox;
  public final AHRS gyro = new AHRS();
  public PathPlannerAuto auto1;
  public final Swerve swerve;
  public final Vision vision;
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    leftJoy = new Joystick(Constants.kControls.LEFT_JOY_ID);
    rightJoy = new Joystick(Constants.kControls.RIGHT_JOY_ID);
    rightJoyButtonTrig = new JoystickButton(rightJoy, 1);
    xbox = new XboxController(2);
    commXbox = new CommandXboxController(2);

    vision = new Vision(gyro);
    
    swerve = new Swerve(gyro, vision);

    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure button bindings
    configureButtonBindings();
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.drive(
      () -> Constants.kControls.Y_DRIVE_LIMITER.calculate(leftJoy.getY()), 
      () -> Constants.kControls.X_DRIVE_LIMITER.calculate(leftJoy.getX()),  
      () -> Constants.kControls.THETA_DRIVE_LIMITER.calculate(rightJoy.getX()),
      true,
      false
      ));

    rightJoyButtonTrig.onTrue(swerve.zeroGyroCommand());
  }
}
