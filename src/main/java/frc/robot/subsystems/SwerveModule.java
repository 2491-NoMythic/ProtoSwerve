// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANCoder m_steerEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_steerPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_STEER_VELOCITY_RADIANS_PER_SECOND, DriveConstants.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannel,
      Double steerEncoderOffset,
      String canivoreName) {
    m_driveMotor = new TalonFX(driveMotorChannel, canivoreName);
    m_steerMotor = new TalonFX(steerMotorChannel, canivoreName);
    m_steerEncoder = new CANCoder(steerEncoderChannel, canivoreName);
    
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();

    // Set the encoder offset to ensure that a measurement of 0 means that the
    // module is facing forwards.
    steerEncoderConfig.magnetOffsetDegrees = steerEncoderOffset;
    // set units of the encoder to radians, with velocity being radians per second.
    steerEncoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
    steerEncoderConfig.unitString = "rad";
    steerEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    // Configure encoder to return -.5 to +.5 rotations.
    steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

    // Apply the configurations.
    m_driveMotor.configAllSettings(driveMotorConfig);
    m_steerEncoder.configAllSettings(steerEncoderConfig);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveSpeedMetersPerSecond(), new Rotation2d(m_steerEncoder.getPosition()));
  }
  /**
   * Returns the current position of the module.
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistanceMeters(), new Rotation2d(m_steerEncoder.getPosition()));
  }
  public Rotation2d getRotation() {
    return new Rotation2d(m_steerEncoder.getPosition());
  }
  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState state =
    SwerveModuleState.optimize(desiredState, new Rotation2d(m_steerEncoder.getPosition()));
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
    m_drivePIDController.calculate(getDriveSpeedMetersPerSecond(), state.speedMetersPerSecond);
    
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
    m_steerPIDController.calculate(m_steerEncoder.getPosition(), state.angle.getRadians());
    
    final double turnFeedforward =
    m_steerFeedforward.calculate(m_steerPIDController.getSetpoint().velocity);
    
    m_driveMotor.set(ControlMode.Current, driveOutput + driveFeedforward);
    m_steerMotor.set(ControlMode.Current, turnOutput + turnFeedforward);
  }

  /**
   * Returns the current encoder velocity of the drive motor.
   * @return The current velocity of the drive motor in meters/second.
   */
  private double getDriveSpeedMetersPerSecond() {
    //                    Encoder ticks per second                    Encoder ticks to meters
    return (m_driveMotor.getSelectedSensorVelocity() * 10) * DriveConstants.DRIVETRAIN_TICKS_TO_METERS;
  }
  /**
   * Returns the current encoder distance of the drive motor.
   * @return The current distance of the drive motor in meters.
   */
  private double getDriveDistanceMeters() {
    return (m_driveMotor.getSelectedSensorPosition() * DriveConstants.DRIVETRAIN_TICKS_TO_METERS);
  }
}