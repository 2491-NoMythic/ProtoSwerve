// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.DriveConstants.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class DrivetrainSubsystem extends SubsystemBase {
	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 5.0;
	
	SwerveDriveKinematics kinematics = DriveConstants.kinematics;

	private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, CANIVORE_DRIVETRAIN);

	/**
	 * These are our modules. We initialize them in the constructor.
	 * 0 = Front Left
	 * 1 = Front Right
	 * 2 = Back Left
	 * 3 = Back Right
	 */
	private final SwerveModule[] modules;

	private final Rotation2d[] lastAngles;

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 5, new Rotation2d(0)),
            List.of(
                    new Translation2d(6, 5),
                    new Translation2d(6, 4)),
            new Pose2d(7, 4, Rotation2d.fromDegrees(180)),
            trajectoryConfig);
			
	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
	private final Field2d m_field = new Field2d();

	public DrivetrainSubsystem() {
		SmartDashboard.putData("Field", m_field);
		
		m_field.getObject("traj").setTrajectory(trajectory);

		modules = new SwerveModule[4];
		lastAngles = new Rotation2d[4];

		modules[0] = new SwerveModule(
				FL_DRIVE_MOTOR_ID,
				FL_STEER_MOTOR_ID,
				FL_STEER_ENCODER_ID,
				FL_STEER_OFFSET,
				CANIVORE_DRIVETRAIN);
		modules[1] = new SwerveModule(
				FR_DRIVE_MOTOR_ID,
				FR_STEER_MOTOR_ID,
				FR_STEER_ENCODER_ID,
				FR_STEER_OFFSET,
				CANIVORE_DRIVETRAIN);
		modules[2] = new SwerveModule(
				BL_DRIVE_MOTOR_ID,
				BL_STEER_MOTOR_ID,
				BL_STEER_ENCODER_ID,
				BL_STEER_OFFSET,
				CANIVORE_DRIVETRAIN);
		modules[3] = new SwerveModule(
				BR_DRIVE_MOTOR_ID,
				BR_STEER_MOTOR_ID,
				BR_STEER_ENCODER_ID,
				BR_STEER_OFFSET,
				CANIVORE_DRIVETRAIN);
	}

	
	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getModulePositions(), new Pose2d(5.0, 5.0, new Rotation2d()));

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		pigeon.setYaw(0.0);
		odometer.resetPosition(new Rotation2d(), getModulePositions(), getPose());
	}

	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(pigeon.getYaw());
	}

	public double getHeading() {
		return Math.IEEEremainder(pigeon.getYaw(), 360);
	}
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
		return positions;
	}
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
		return states;
	}
	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }
	/**
	 *  Sets the modules speed and rotation to zero.
	 */
	public void pointWheelsForward() {
		for (int i = 0; i < 4; i++) {
			setModule(i, new SwerveModuleState(0, new Rotation2d()));
		}
	}
	public void drive(ChassisSpeeds new_ChassisSpeeds) {
		chassisSpeeds = new_ChassisSpeeds;
	}
	/**
	 * Sets all module drive speeds to 0, but leaves the wheel angles where they were.
	 */
	public void stop() {
		for (int i = 0; i < 4; i++) {
			setModule(i, new SwerveModuleState(0, lastAngles[i]));
		}
	}
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
		for (int i = 0; i < 4; i++) {
			setModule(i, desiredStates[i]);
		}
	}
	private void setModule(int i, SwerveModuleState desiredState) {
		modules[i].setDesiredState(desiredState);
		lastAngles[i] = desiredState.angle;
	}

	@Override
	public void periodic() {
		SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
		// FIXME test the speedMetersPerSecond values to see if we also have to check for reversed speeds, 
		// eg. maxSpeed = 5, currentSpeed = -8
		double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
		if (maxSpeed <= 0.01) {
			stop();
		} else {
			setModuleStates(desiredStates);
		}

		odometer.update(getGyroscopeRotation(), getModulePositions());
		m_field.setRobotPose(odometer.getPoseMeters());

        SmartDashboard.putNumber("Robot Angle", pigeon.getYaw());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
	}
}