// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.Constants.Drivetrain.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase{
  public static final double kMaxSpeed = K_MAX_SPEED;
  public static final double kMaxAngularSpeed = K_MAX_ANGULAR_SPEED;

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeftModule = new SwerveModule(FL_DRIVE_MOTOR_ID, FL_TURN_MOTOR_ID, 0, 1, 2, 3);
  private final SwerveModule frontRightModule = new SwerveModule(FR_DRIVE_MOTOR_ID, FR_TURN_MOTOR_ID, 4, 5, 6, 7);
  private final SwerveModule backLeftModule = new SwerveModule(BL_DRIVE_MOTOR_ID, BL_TURN_MOTOR_ID, 8, 9, 10, 11);
  private final SwerveModule backRightModule = new SwerveModule(BR_DRIVE_MOTOR_ID, BR_TURN_MOTOR_ID, 12, 13, 14, 15);

  private final AnalogGyro gyro = new AnalogGyro(GYRO_ID);

  private final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry swerveOdometry =
      new SwerveDriveOdometry(swerveKinematics, gyro.getRotation2d());

  public Drivetrain() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param forwardSpeed Speed of the robot in the x direction (forward).
   * @param sidewaysSpeed Speed of the robot in the y direction (sideways).
   * @param rotation Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double forwardSpeed, double sidewaysSpeed, double rotation, boolean fieldRelative) {
    var swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed, rotation, gyro.getRotation2d())
                : new ChassisSpeeds(forwardSpeed, sidewaysSpeed, rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    swerveOdometry.update(
        gyro.getRotation2d(),
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    );
  }
}