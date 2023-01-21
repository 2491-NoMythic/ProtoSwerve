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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import frc.robot.CTREConfigs;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;
  private final Rotation2d m_steerEncoderOffset;
  private double m_desiredSteerAngle;
  private double m_desiredDriveSpeed;

  private GenericEntry m_driveP;
  private GenericEntry m_driveI;
  private GenericEntry m_driveD;
  private GenericEntry m_driveS;
  private GenericEntry m_driveV;

  private GenericEntry m_steerP;
  private GenericEntry m_steerI;
  private GenericEntry m_steerD;
  private GenericEntry m_steerS;
  private GenericEntry m_steerV;


  // // Gains are for example purposes only - must be determined for your own robot!
  // private final PIDController m_drivePIDController = new PIDController(
  //     DriveConstants.K_DRIVE_P,
  //     DriveConstants.K_DRIVE_I,
  //     DriveConstants.K_DRIVE_D);

  // // Gains are for example purposes only - must be determined for your own robot!
  // private final ProfiledPIDController m_steerPIDController =
  //     new ProfiledPIDController(
  //         DriveConstants.K_STEER_P,
  //         DriveConstants.K_STEER_I,
  //         DriveConstants.K_STEER_D,
  //         new TrapezoidProfile.Constraints(
  //             DriveConstants.MAX_STEER_VELOCITY_RADIANS_PER_SECOND, DriveConstants.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  // // Gains are for example purposes only - must be determined for your own robot!
  // private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(DriveConstants.K_DRIVE_FF_S, DriveConstants.K_DRIVE_FF_V);
  // private final SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(DriveConstants.K_STEER_FF_S, DriveConstants.K_STEER_FF_V);

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

    // Apply the configurations.
    m_driveMotor.getConfigurator().apply(DrivetrainSubsystem.ctreConfig.driveMotorConfig);
    m_steerMotor.getConfigurator().apply(DrivetrainSubsystem.ctreConfig.steerMotorConfig);
    m_steerEncoder.getConfigurator().apply(DrivetrainSubsystem.ctreConfig.steerEncoderConfig);
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
      int driveMotorChannel,
      int steerMotorChannel,
      int steerEncoderChannel,
      Rotation2d steerEncoderOffset,
      String canivoreName) {
    this(moduleName, driveMotorChannel, steerMotorChannel, steerEncoderChannel, steerEncoderOffset, canivoreName);
    
    // Make grid layouts.
    ShuffleboardLayout GeneralInfo = sbLayout.getLayout("GeneralInfo", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 1, "Number of rows", 3, "Label position", "Top"));

    ShuffleboardLayout PIDInfo = sbLayout.getLayout("PIDInfo", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "Top"));

    ShuffleboardLayout ModuleInfo = sbLayout.getLayout("ModuleInfo", BuiltInLayouts.kGrid)
      .withProperties(Map.of("Number of columns", 1, "Number of rows", 4, "Label position", "Top"));

    // ShuffleboardLayout DriveInfo = PIDInfo.getLayout("DriveInfo", BuiltInLayouts.kGrid)
    //   .withProperties(Map.of("Number of columns", 2, "Number of rows", 3, "Label position", "Top"));

    // ShuffleboardLayout SteerInfo = PIDInfo.getLayout("SteerInfo", BuiltInLayouts.kGrid)
    //   .withProperties(Map.of("Number of columns", 2, "Number of rows", 3, "Label position", "Top"));

    GeneralInfo.add(moduleName + " Angle", new GyroView(() -> getRotation().getDegrees()))
      .withWidget(BuiltInWidgets.kGyro) 
      .withProperties(Map.of("Major tick spacing", 180, "Counter Clockwise", true));
    GeneralInfo.addNumber(moduleName + " Speed", () -> getSpeedMetersPerSecond());

    ModuleInfo.addNumber(moduleName + " Speed", () -> getSpeedMetersPerSecond());
    ModuleInfo.addNumber(moduleName + " Angle", () -> getRotation().getDegrees());
    ModuleInfo.addNumber(moduleName + " Speed Target", () -> getTargetSpeedMetersPerSecond());
    ModuleInfo.addNumber(moduleName + " Angle Target", () -> getTargetAngle());

    // m_driveP = DriveInfo.add("Drive P", DriveConstants.K_DRIVE_P).getEntry();
    // m_driveI = DriveInfo.addPersistent("Drive I", DriveConstants.K_DRIVE_I).getEntry();
    // m_driveD = DriveInfo.addPersistent("Drive D", DriveConstants.K_DRIVE_D).getEntry();
    // m_driveS = DriveInfo.addPersistent("Drive FFs", DriveConstants.K_DRIVE_FF_S).getEntry();
    // m_driveV = DriveInfo.addPersistent("Drive FFv", DriveConstants.K_DRIVE_FF_V).getEntry();
    // DriveInfo.addNumber("Drive Error", () -> m_drivePIDController.getPositionError());

    // m_steerP = SteerInfo.addPersistent("Steer P", DriveConstants.K_STEER_P).getEntry();
    // m_steerI = SteerInfo.addPersistent("Steer I", DriveConstants.K_STEER_I).getEntry();
    // m_steerD = SteerInfo.addPersistent("Steer D", DriveConstants.K_STEER_D).getEntry();
    // m_steerS = SteerInfo.addPersistent("Steer FFs", DriveConstants.K_STEER_FF_S).getEntry();
    // m_steerV = SteerInfo.addPersistent("Steer FFv", DriveConstants.K_STEER_FF_V).getEntry();
    // SteerInfo.addNumber("Drive Error", () -> m_steerPIDController.getPositionError());
    // PIDInfo.add(m_drivePIDController);
    // PIDInfo.add(m_steerPIDController);
    // PIDInfo.addNumber("PIDError", ()-> m_drivePIDController.getPositionError());
  }
  // /**
  //  * Sets the PID and FeedForward values to both of the PID loops.
  //  * @return
  //  */
  // public void SetPID(){
  //   m_drivePIDController.setPID(
  //     m_driveP.getDouble(DriveConstants.K_DRIVE_P), 
  //     m_driveI.getDouble(DriveConstants.K_DRIVE_I), 
  //     m_driveD.getDouble(DriveConstants.K_DRIVE_D)
  //     );
  //   m_steerPIDController.setPID(
  //     m_steerP.getDouble(DriveConstants.K_STEER_P), 
  //     m_steerI.getDouble(DriveConstants.K_STEER_I), 
  //     m_steerD.getDouble(DriveConstants.K_STEER_D)
  //     );
  // }
  public void resetToAbsolute(){
    double absolutePosition = m_steerEncoder.getAbsolutePosition().getValue() - m_steerEncoderOffset.getRotations();
    m_steerMotor.setRotorPosition(absolutePosition);
  }
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
    return Rotation2d.fromRotations(m_steerMotor.getPosition().getValue());
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

  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    if (desiredState.angle == null) {
      DriverStation.reportWarning("Cannot set module angle to null.", true);
    }
    m_desiredSteerAngle = desiredState.angle.getDegrees();
    m_desiredDriveSpeed = desiredState.speedMetersPerSecond;
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState state =
    SwerveModuleState.optimize(desiredState, getRotation());
    
    // // Calculate the drive output from the drive PID controller.
    // final double driveOutput =
    // m_drivePIDController.calculate(getSpeedMetersPerSecond(), state.speedMetersPerSecond);
    
    // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
    // // Calculate the turning motor output from the turning PID controller.
    // final double turnOutput =
    // m_steerPIDController.calculate(m_steerEncoder.getPosition(), state.angle.getRadians());
    
    // final double turnFeedforward =
    // m_steerFeedforward.calculate(m_steerPIDController.getSetpoint().velocity);
    
    // m_driveMotor.set(ControlMode.Current, driveOutput + driveFeedforward);
    // m_steerMotor.set(ControlMode.Current, turnOutput + turnFeedforward);
  }

}