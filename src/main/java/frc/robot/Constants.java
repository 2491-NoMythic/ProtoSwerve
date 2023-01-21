// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import com.ctre.phoenixpro.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        private DriveConstants() {
        }
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4953;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *  
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4953;

        /**
         * The diameter of the module's wheel in meters.
         */
        public static final double DRIVETRAIN_WHEEL_DIAMETER = 0.10033;

        /**
         * The overall drive reduction of the module. Multiplying motor rotations by
         * this value should result in wheel rotations.
         */
        public static final double DRIVETRAIN_DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

        /**
         * Whether the drive motor should be counterclockwise or clockwise positive. 
         * If there is an odd number of gear reductions this is typically clockwise-positive.
         */
        public static final InvertedValue DRIVETRAIN_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;

        /**
         * The overall steer reduction of the module. Multiplying motor rotations by
         * this value should result in wheel rotations.
         */
        public static final double DRIVETRAIN_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

        /**
         * Whether the steer motor should be counterclockwise or clockwise positive. 
         * If there is an odd number of gear reductions this is typically clockwise-positive.
         */
        public static final InvertedValue DRIVETRAIN_STEER_INVERTED = InvertedValue.CounterClockwise_Positive;

        /**
         * How many meters the wheels travel per rotation. Multiply rotations by this to get meters.
         */
        public static final double DRIVETRAIN_ROTATIONS_TO_METERS = (DRIVETRAIN_WHEEL_DIAMETER * Math.PI);

        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        /*
         * FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
         * The formula for calculating the theoretical maximum velocity is:
         * <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            DRIVETRAIN_DRIVE_REDUCTION * DRIVETRAIN_WHEEL_DIAMETER * Math.PI;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
            
            public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        public static final String DRIVETRAIN_SMARTDASHBOARD_TAB = "Drivetrain";
        public static final String CANIVORE_DRIVETRAIN = "Swerve";
        public static final int DRIVETRAIN_PIGEON_ID = 0;

        public static final int FL_DRIVE_MOTOR_ID = 1;
        public static final int FL_STEER_MOTOR_ID = 2;
        public static final int FL_STEER_ENCODER_ID = 1;
        public static final Rotation2d FL_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final int FR_DRIVE_MOTOR_ID = 3;
        public static final int FR_STEER_MOTOR_ID = 4;
        public static final int FR_STEER_ENCODER_ID = 2;
        public static final Rotation2d FR_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final int BL_DRIVE_MOTOR_ID = 5;
        public static final int BL_STEER_MOTOR_ID = 6;
        public static final int BL_STEER_ENCODER_ID = 3;
        public static final Rotation2d BL_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final int BR_DRIVE_MOTOR_ID = 7;
        public static final int BR_STEER_MOTOR_ID = 8;
        public static final int BR_STEER_ENCODER_ID = 4;
        public static final Rotation2d BR_STEER_OFFSET = Rotation2d.fromRotations(0);

        public static final double K_MAX_SPEED = 3.0; // 3 meters per second
        public static final double K_MAX_ANGULAR_SPEED = Math.PI; // 1/2 radians rotation per second

        public static final double K_TURN_P = 0.015;
        public static final double K_TURN_I = 0.0;
        public static final double K_TURN_D = 0.0;
        /** This tuning parameter indicates how close to "on target" the PID Controller will attempt to get.*/
        public static final double K_TURN_TOLORANCE_DEGREES = 2.0;
        public static final double K_TURN_TOLORANCE_DEG_PER_SEC = 10;


        // Drive Motor
        public static final double K_DRIVE_P = 1;
        public static final double K_DRIVE_I = 0;
        public static final double K_DRIVE_D = 0;
        public static final double K_DRIVE_FF_S = 1; 
        public static final double K_DRIVE_FF_V = 3;

        // Steer Motor
        /**
         * The maximum velocity of the steer motor. <p> 
         * This is the limit of how fast the wheels can rotate in place.
         */
        public static final double MAX_STEER_VELOCITY_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second.
        /**
         * The maximum acceleration of the steer motor. <p>
         * This is the limit of how fast the wheels can change rotation speed.
         */
        public static final double MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; 
        public static final double K_STEER_P = 0.2;
        public static final double K_STEER_I = 0;
        public static final double K_STEER_D = 0; 
        public static final double K_STEER_FF_S = 0; 
        public static final double K_STEER_FF_V = 0; 
    }
    public final class PS4 {
        private PS4() {
        }
        public static final int CONTROLLER_ID = 1;
        /**Left Stick Tilt Left/Right*/
        public static final int X_AXIS = 1; 
        /**Left Stick Tilt Forwards/Backwards*/
        public static final int Y_AXIS = 0;
        /**Right Stick Tilt Left/Right*/
        public static final int Z_AXIS = 2;
        public static final int Z_ROTATE = 5;
        public static final int SLIDER_AXIS = 3;
        public static final int TRIGGER_BUTTON = 0;
        public static final double NO_INPUT = 404;
        public static final double DEADBAND_NORMAL = 0.05;
        public static final double DEADBAND_LARGE = 0.1;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.MAX_VELOCITY_METERS_PER_SECOND / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond =
                DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kinematics);
    }
}

