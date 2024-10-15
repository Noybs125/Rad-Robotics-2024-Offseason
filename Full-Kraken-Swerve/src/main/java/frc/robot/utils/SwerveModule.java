package frc.robot.utils;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModule {
  public final int moduleNumber;

  private final TalonFX driveMotor;
  private final PhoenixPIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;
  private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  private final TalonFX angleMotor;
  private final PhoenixPIDController anglePID;
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
  
  private final CANcoder canCoder;
  private final double canCoderOffsetDegrees;
  private final CANcoderConfigurator canConfig;

  private double lastAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new TalonFX(constants.driveMotorID);
    drivePID = new PhoenixPIDController(Constants.kSwerve.DRIVE_KP, Constants.kSwerve.DRIVE_KI, Constants.kSwerve.DRIVE_KD);
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new TalonFX(constants.angleMotorID);

    anglePID = new PhoenixPIDController(Constants.kSwerve.ANGLE_KP, Constants.kSwerve.ANGLE_KI, Constants.kSwerve.ANGLE_KD);

    canCoder = new CANcoder(constants.canCoderID);
    canCoderOffsetDegrees = constants.canCoderOffsetDegrees;
    canConfig = canCoder.getConfigurator();

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    // System.out.println("Angle: " + state.angle.getRadians() + "Mod #: " + moduleNumber);
    state = SwerveModuleState.optimize(state, getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      //drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    } else {
      driveVelocity.Velocity = state.speedMetersPerSecond / Constants.kSwerve.WHEEL_CIRCUMFERENCE;
      driveVelocity.FeedForward = driveFeedforward.calculate(state.speedMetersPerSecond);
      driveMotor.setControl(driveVelocity);
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle.getRadians();

    angleMotor.setControl(anglePosition.withPosition(state.angle.getRadians()));

    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveMotor.getVelocity().getValueAsDouble();
    Rotation2d rot = new Rotation2d(angleMotor.getPosition().getValueAsDouble() * Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
    return new SwerveModuleState(velocity, rot);
  }

  public double getCanCoder() {
    return canCoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(angleMotor.getPosition().getValueAsDouble() * Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
  }

  public SwerveModulePosition getPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble();
    Rotation2d rot = new Rotation2d(angleMotor.getPosition().getValueAsDouble() * Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
    return new SwerveModulePosition(distance, rot);
  }

  public Rotation2d getCanCoderDegrees(){
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition().getValueAsDouble() * 360);
  }
  
  private void configureDevices() {
    // CanCoder configuration.
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    canConfig.apply(canCoderConfiguration);
  
    // Drive motor configuration.

    

    driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kSwerve.OPEN_LOOP_RAMP;
    driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kSwerve.CLOSED_LOOP_RAMP;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kSwerve.DRIVE_CURRENT_LIMIT; // change the constant
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.kSwerve.DRIVE_CURRENT_LIMIT;

    driveMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION);
    driveMotor.setNeutralMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    
    
    drivePID.setP(Constants.kSwerve.DRIVE_KP);
    drivePID.setI(Constants.kSwerve.DRIVE_KI);
    drivePID.setD(Constants.kSwerve.DRIVE_KD);
    
 
    driveMotor.setPosition(0);

    

    // Angle motor configuration.



    angleMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kSwerve.ANGLE_CURRENT_LIMIT;
  
    angleMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
    angleMotor.setNeutralMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    

    anglePID.setP(Constants.kSwerve.ANGLE_KP);
    anglePID.setI(Constants.kSwerve.ANGLE_KI);
    anglePID.setD(Constants.kSwerve.ANGLE_KD);


    
 
    angleMotor.setPosition(Units.degreesToRadians((canCoder.getAbsolutePosition().getValueAsDouble() * 360) - canCoderOffsetDegrees)); // added ".getValue..."
  }
}
