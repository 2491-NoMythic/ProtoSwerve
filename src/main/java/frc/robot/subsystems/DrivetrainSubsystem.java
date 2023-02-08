// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.DriveConstants.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class DrivetrainSubsystem extends SubsystemBase {
	public static final CTREConfigs ctreConfig = new CTREConfigs();
	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 5.0;
	
	public SwerveDriveKinematics kinematics = DriveConstants.kinematics;

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

    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //         AutoConstants.kMaxSpeedMetersPerSecond,
    //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //                 .setKinematics(DriveConstants.kinematics);

    // // 2. Generate trajectory
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(5, 5, new Rotation2d(0)),
    //         List.of(
    //                 new Translation2d(6, 5),
    //                 new Translation2d(6, 4)),
    //         new Pose2d(7, 4, Rotation2d.fromDegrees(180)),
    //         trajectoryConfig);
			
	private ChassisSpeeds chassisSpeeds;
	private final SwerveDriveOdometry odometer;

	public final Field2d m_field = new Field2d();

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab(DRIVETRAIN_SMARTDASHBOARD_TAB);
		tab.add("Field", m_field)
			.withWidget(BuiltInWidgets.kField)
			.withSize(5, 3)
			.withPosition(8, 0);
		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putData("resetOdometry", new InstantCommand(() -> this.resetOdometry()));
		// m_field.getObject("traj").setTrajectory(trajectory);
		
		
		modules = new SwerveModule[4];
		lastAngles = new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()}; // manually make empty angles to avoid null errors.
		
		modules[0] = new SwerveModule(
			"FL",
			tab.getLayout("Front Left Module", BuiltInLayouts.kGrid)
				.withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"))
				.withSize(4, 3)
				.withPosition(0, 0),
			true,
			FL_DRIVE_MOTOR_ID,
			FL_STEER_MOTOR_ID,
			FL_STEER_ENCODER_ID,
			FL_STEER_OFFSET,
			CANIVORE_DRIVETRAIN);
		modules[1] = new SwerveModule(
			"FR",
			tab.getLayout("Front Right Module", BuiltInLayouts.kGrid)
				.withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"))
				.withSize(4, 3)
				.withPosition(4, 0),
				false,
			FR_DRIVE_MOTOR_ID,
			FR_STEER_MOTOR_ID,
			FR_STEER_ENCODER_ID,
			FR_STEER_OFFSET,
			CANIVORE_DRIVETRAIN);
		modules[2] = new SwerveModule(
			"BL",
			tab.getLayout("Back Left Module", BuiltInLayouts.kGrid)
				.withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"))
				.withSize(4, 3)
				.withPosition(0, 3),
				false,
				BL_DRIVE_MOTOR_ID,
				BL_STEER_MOTOR_ID,
				BL_STEER_ENCODER_ID,
				BL_STEER_OFFSET,
				CANIVORE_DRIVETRAIN);
		modules[3] = new SwerveModule(
			"BR",
			tab.getLayout("Back Right Module", BuiltInLayouts.kGrid)
				.withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"))
				.withSize(4, 3)
				.withPosition(4, 3),
				false,
			BR_DRIVE_MOTOR_ID,
			BR_STEER_MOTOR_ID,
			BR_STEER_ENCODER_ID,
			BR_STEER_OFFSET,
			CANIVORE_DRIVETRAIN);
		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		// calibrateWheels();
		odometer = new SwerveDriveOdometry(
				kinematics, 
				getGyroscopeRotation(),
				getModulePositions(),
				DRIVE_ODOMETRY_ORIGIN);
		}
	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		// calibrateWheels();
		pigeon.setYaw(0.0);
		odometer.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
	}
	public void zeroGyroscope(double angleDeg) {
		// calibrateWheels();
		pigeon.setYaw(angleDeg);
		odometer.resetPosition(new Rotation2d().fromDegrees(angleDeg), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d().fromDegrees(angleDeg)));
	}

	// public void calibrateWheels() {
	// 	for (int i = 0; i < 4; i++) {
	// 		modules[i].resetToAbsolute();
	// 	};
	// }
	public Rotation2d getGyroscopeRotation() {//todo make continuous vs not continuous versions
		return pigeon.getRotation2d();
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
    public void resetOdometry() {
        odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), new Pose2d(DRIVE_ODOMETRY_ORIGIN.getTranslation(), getGyroscopeRotation()));
    }
	
public Command followPPTrajectory(PathPlannerTrajectory traj, boolean isFirstPath) {
	return new SequentialCommandGroup(
		 new InstantCommand(() -> {
		   // Reset odometry for the first path you run during auto
		   if(isFirstPath){
			   this.resetOdometry(traj.getInitialHolonomicPose());
		   }
		 }),
		 new PPSwerveControllerCommand(
			 traj, 
			 this::getPose, // Pose supplier
			 this.kinematics, // SwerveDriveKinematics
			 new PIDController(// X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
			 DriveConstants.k_XY_P,
			 DriveConstants.k_XY_I,
			 DriveConstants.k_XY_D),
			  new PIDController(// Y controller (usually the same values as X controller)
			  DriveConstants.k_XY_P,
			  DriveConstants.k_XY_I,
			  DriveConstants.k_XY_D),
			  new PIDController(// Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
			  DriveConstants.k_THETA_P,
			  DriveConstants.k_THETA_I,
			  DriveConstants.k_THETA_D),
			 this::setModuleStates, // Module states consumer
			 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			 this // Requires this drive subsystem
		 ),
		 new InstantCommand(() -> this.stop())
	 );
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
		if (maxSpeed <= DriveConstants.DRIVE_DEADBAND_MPS) {
			stop();
		} else {
			setModuleStates(desiredStates);
		}

		odometer.update(getGyroscopeRotation(), getModulePositions());
		m_field.setRobotPose(odometer.getPoseMeters());

        SmartDashboard.putNumber("Robot Angle", getGyroscopeRotation().getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
	}
}