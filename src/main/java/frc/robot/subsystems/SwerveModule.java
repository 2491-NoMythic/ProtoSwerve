// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.CoastOut;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;
  private final Rotation2d m_steerEncoderOffset;

  ShuffleboardTab debugInfo;
  /**
   * The target wheel angle in rotations. [-.5, .5]
   */
  private double m_desiredSteerAngle;
  /**
   * The target wheel speed in rotations per second
   */
  private double m_desiredDriveSpeed;

  private VelocityDutyCycle m_driveControl = new VelocityDutyCycle(0, true, 0, 0, false);
  private PositionDutyCycle m_steerControl = new PositionDutyCycle(0, true, 0, 0, false);
  private CoastOut m_coastControl = new CoastOut();
  private StaticBrake m_brakeControl = new StaticBrake();
  private NeutralOut m_neutralControl = new NeutralOut();

  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning encoder.
   *
   * @param driveMotorChannel 
   * @param steerMotorChannel 
   * @param steerEncoderChannel
   * @param steerEncoderOffset
   * @param canivoreName
   */
  public SwerveModule(
      String moduleName,
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannel,
      Rotation2d steerEncoderOffset,
      String canivoreName) {
    m_driveMotor = new TalonFX(driveMotorChannel, canivoreName);
    m_steerMotor = new TalonFX(steerMotorChannel, canivoreName);
    m_steerEncoder = new CANcoder(steerEncoderChannel, canivoreName);
    m_steerEncoderOffset = steerEncoderOffset;
    
    // com.ctre.phoenix.motorcontrol.can.TalonFX testmotor = new com.ctre.phoenix.motorcontrol.can.TalonFX(1);
   

    CANcoderConfiguration steerEncoderConfig = DrivetrainSubsystem.ctreConfig.steerEncoderConfig;
    TalonFXConfiguration steerMotorConfig = DrivetrainSubsystem.ctreConfig.steerMotorConfig;

    steerEncoderConfig.MagnetSensor.MagnetOffset = -m_steerEncoderOffset.getRotations();
    steerMotorConfig.Feedback.FeedbackRemoteSensorID = steerEncoderChannel;
    // Apply the configurations.
    m_driveMotor.getConfigurator().apply(DrivetrainSubsystem.ctreConfig.driveMotorConfig);
    m_steerMotor.getConfigurator().apply(steerMotorConfig);
    m_steerEncoder.getConfigurator().apply(steerEncoderConfig);
    }

  static class GyroView implements Sendable {
    DoubleSupplier value;
    GyroView(DoubleSupplier value) {
      this.value = value;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Gyro");
      builder.addDoubleProperty("Value", value, null);
    }
  }
  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning encoder. <br>
   * Also impliments a shuffleBoard tab for PID tuning and debugging.
   *
   * @param sbLayout The shuffleBoard layout to display info about this module.
   * @param driveMotorChannel 
   * @param steerMotorChannel 
   * @param steerEncoderChannel
   * @param steerEncoderOffset
   * @param canivoreName
   */
  public SwerveModule(
      String moduleName,
      ShuffleboardLayout sbLayout,
      boolean debug,
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannel,
      Rotation2d steerEncoderOffset,
      String canivoreName) {
    this(moduleName, driveMotorChannel, steerMotorChannel, steerEncoderChannel, steerEncoderOffset, canivoreName);
    
    // Make grid layouts.
    ShuffleboardLayout MotorInfo = sbLayout.getLayout("GeneralInfo", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 1, "Number of rows", 3, "Label position", "LEFT"));

    ShuffleboardLayout ModuleInfo = sbLayout.getLayout("ModuleInfo", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 1, "Number of rows", 4, "Label position", "RIGHT"));

    MotorInfo.add(moduleName + " Angle", new GyroView(() -> MathUtil.inputModulus(getRotation().getDegrees(), -180.0, 180.0)))
      .withWidget(BuiltInWidgets.kGyro) 
      .withProperties(Map.of("Major tick spacing", 180, "Counter Clockwise", true));
    MotorInfo.addNumber(moduleName + " Drive Voltage", () -> m_driveMotor.getStatorCurrent().getValue());
    MotorInfo.addNumber(moduleName + " Steer Voltage", () -> m_steerMotor.getStatorCurrent().getValue());
    
    ModuleInfo.addNumber(moduleName + " Speed", () -> getSpeedMetersPerSecond());
    ModuleInfo.addNumber(moduleName + " Angle", () -> getRotation().getDegrees());
    ModuleInfo.addNumber(moduleName + " Speed Target", () -> getTargetSpeedMetersPerSecond());
    ModuleInfo.addNumber(moduleName + " Angle Target", () -> getTargetAngle());
    if (debug) {
      debugInfo = Shuffleboard.getTab("Debug");
      debugInfo.addNumber("Drive Target", () -> m_driveMotor.getClosedLoopReference().getValue());
      debugInfo.addNumber("Drive Value", () -> m_driveMotor.getVelocity().getValue());
      debugInfo.addNumber("Drive Error", () -> m_driveMotor.getClosedLoopError().getValue());

      debugInfo.addNumber("Steer Target", () -> m_steerMotor.getClosedLoopReference().getValue());
      debugInfo.addNumber("Steer Value", () -> getRotation().getRotations());
      debugInfo.addNumber("Steer Error", () -> m_steerMotor.getClosedLoopError().getValue());
      debugInfo.add(m_steerMotor);
    }

  }

  // public void resetToAbsolute(){
  //   double absolutePosition = m_steerEncoder.getAbsolutePosition().getValue() -m_steerEncoderOffset.getRotations();
  //   m_steerMotor.setRotorPosition(absolutePosition);
  // }
  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getSpeedMetersPerSecond(), getRotation());
  }
  /**
   * Returns the current position of the module.
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistanceMeters(), getRotation());
  }
    /**
   * Returns the current encoder angle of the steer motor.
   * @return The current encoder angle of the steer motor.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(MathUtil.inputModulus(m_steerMotor.getPosition().getValue(), -0.5, 0.5));
  }
  /**
   * Returns the target angle of the wheel.
   * @return The target angle of the wheel in degrees.
   */
  public double getTargetAngle() {
    return m_desiredSteerAngle;
  }
  /**
   * Returns the target speed of the wheel.
   * @return The target speed of the wheel in meters/second.
   */
  public double getTargetSpeedMetersPerSecond() {
    return m_desiredDriveSpeed;
  }
  /**
   * Returns the current encoder velocity of the drive motor.
   * @return The current velocity of the drive motor in meters/second.
   */
  public double getSpeedMetersPerSecond() {
    return (m_driveMotor.getVelocity().getValue() * DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }
  /**
   * Returns the current encoder distance of the drive motor.
   * @return The current distance of the drive motor in meters.
   */
  public double getDriveDistanceMeters() {
    return (m_driveMotor.getPosition().getValue() * DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS);
  }

  public void stop() {

  }
  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    if (desiredState.angle == null) {
      DriverStation.reportWarning("Cannot set module angle to null.", true);
    }
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState state =
    SwerveModuleState.optimize(desiredState, getRotation());

    m_desiredSteerAngle = MathUtil.inputModulus(state.angle.getRotations(), -0.5, 0.5);
    m_desiredDriveSpeed = state.speedMetersPerSecond / DriveConstants.DRIVETRAIN_ROTATIONS_TO_METERS;

    m_driveMotor.setControl(m_driveControl.withVelocity(m_desiredDriveSpeed).withFeedForward(m_desiredDriveSpeed/DriveConstants.MAX_VELOCITY_RPS_EMPIRICAL));
    // m_driveMotor.setControl(new DutyCycleOut(0.5));
    m_steerMotor.setControl(m_steerControl.withPosition(m_desiredSteerAngle));
    // m_steerMotor.setControl(m_brakeControl);
  }

}