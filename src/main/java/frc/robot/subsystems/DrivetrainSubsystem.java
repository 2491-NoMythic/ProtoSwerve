// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Drivetrain.*;

import java.util.Arrays;
import java.util.Collections;

public class DrivetrainSubsystem extends SubsystemBase {
	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 5.0;
	// FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
	//  The formula for calculating the theoretical maximum velocity is:
	//   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
	//  By default this value is setup for a Mk4 standard module using Falcon500s to drive.
	//  An example of this constant for a Mk4 L2 module with NEOs to drive is:
	//   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
	/**
	 * The maximum velocity of the robot in meters per second.
	 * <p>
	 * This is a measure of how fast the robot should be able to drive in a straight line.
	 */
	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
		SdsModuleConfigurations.MK4_L1.getDriveReduction() *
		SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
		Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		// Front left
		new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Front right
		new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Back left
		new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Back right
		new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
	);

	// By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
	// The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
	// cause the angle reading to increase until it wraps back over to zero.
	private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

	/**
	 * These are our modules. We initialize them in the constructor.
	 * 0 = Front Left
	 * 1 = Front Right
	 * 2 = Back Left
	 * 3 = Back Right
	 */
	private final SwerveModule[] modules;

	private final double[] lastAngles;

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
	private final Field2d m_field = new Field2d();

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
		
		// tab.add("Field", m_field);
		SmartDashboard.putData("Field", m_field);
		
		modules = new SwerveModule[4];
		lastAngles = new double[4];

		modules[0] = Mk4SwerveModuleHelper.createFalcon500(
			// This parameter is optional, but will allow you to see the current state of the module on the dashboard.
			tab.getLayout("Front Left Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(0, 0),
			// This can either be STANDARD or FAST depending on your gear configuration
			Mk4SwerveModuleHelper.GearRatio.L1,// FIXME find the actual gear ratio
			// This is the ID of the drive motor
			FL_DRIVE_MOTOR_ID,
			// This is the ID of the steer motor
			FL_STEER_MOTOR_ID,
			// This is the ID of the steer encoder
			FL_STEER_ENCODER_ID,
			// This is how much the steer encoder is offset from true  (In our case, zero is facing straight forward)
			FL_STEER_OFFSET
		);

		// We will do the same for the other modules
		modules[1] = Mk4SwerveModuleHelper.createFalcon500(
			tab.getLayout("Front Right Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(2, 0),
			Mk4SwerveModuleHelper.GearRatio.L1,// FIXME find the actual gear ratio
			FR_DRIVE_MOTOR_ID,
			FR_STEER_MOTOR_ID,
			FR_STEER_ENCODER_ID,
			FR_STEER_OFFSET
		);

		modules[2] = Mk4SwerveModuleHelper.createFalcon500(
			tab.getLayout("Back Left Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(4, 0),
			Mk4SwerveModuleHelper.GearRatio.L1,// FIXME find the actual gear ratio
			BL_DRIVE_MOTOR_ID,
			BL_STEER_MOTOR_ID,
			BL_STEER_ENCODER_ID,
			BL_STEER_OFFSET
		);

		modules[3] = Mk4SwerveModuleHelper.createFalcon500(
			tab.getLayout("Back Right Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(6, 0),
			Mk4SwerveModuleHelper.GearRatio.L1,// FIXME find the actual gear ratio
			BR_DRIVE_MOTOR_ID,
			BR_STEER_MOTOR_ID,
			BR_STEER_ENCODER_ID,
			BR_STEER_OFFSET
		);
	}


	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), new Pose2d(5.0, 5.0, new Rotation2d()));

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		pigeon.setYaw(0.0);
		odometer.resetPosition(new Pose2d(5.0, 5.0, new Rotation2d()), new Rotation2d());
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

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }
	
	public void pointWheelsForward() {
		for (int i = 0; i < 4; i++) {
			setModule(i, 0, 0);
		}
	}

	public void drive(ChassisSpeeds new_ChassisSpeeds) {
		chassisSpeeds = new_ChassisSpeeds;
	}

	private void setModule(int i, double driveVoltage, double steerAngle) {
		modules[i].set(driveVoltage, steerAngle);
		lastAngles[i] = steerAngle;
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
		double maxSpeed = Collections.max(Arrays.asList(states)).speedMetersPerSecond;
		if (maxSpeed <= 0.01) {
			for (int i = 0; i < 4; i++) {
				setModule(i, 0, lastAngles[i]);
			}
		} else {
			SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND); //changed normalizeWheelSpeeds to desaturateWheelSpeeds
			for (int i = 0; i < 4; i++) {
				setModule(i, states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[i].angle.getRadians());
			}
		}

		odometer.update(getGyroscopeRotation(), 
			new SwerveModuleState(modules[0].getDriveVelocity(), new Rotation2d(modules[0].getSteerAngle())), 
			new SwerveModuleState(modules[1].getDriveVelocity(), new Rotation2d(modules[1].getSteerAngle())), 
			new SwerveModuleState(modules[2].getDriveVelocity(), new Rotation2d(modules[2].getSteerAngle())),
			new SwerveModuleState(modules[3].getDriveVelocity(), new Rotation2d(modules[3].getSteerAngle()))
		);


		m_field.setRobotPose(odometer.getPoseMeters());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
	}
}