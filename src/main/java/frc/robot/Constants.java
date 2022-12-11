// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public static final class Drivetrain {
        private Drivetrain() {
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

        public static final String CANIVORE_DRIVETRAIN = "Swerve";
        public static final int DRIVETRAIN_PIGEON_ID = 0;

        public static final int FL_DRIVE_MOTOR_ID = 1;
        public static final int FL_STEER_MOTOR_ID = 2;
        public static final int FL_STEER_ENCODER_ID = 1;
        public static final double FL_STEER_OFFSET = -Math.toRadians(48.9);

        public static final int FR_DRIVE_MOTOR_ID = 3;
        public static final int FR_STEER_MOTOR_ID = 4;
        public static final int FR_STEER_ENCODER_ID = 2;
        public static final double FR_STEER_OFFSET = -Math.toRadians(88.38);

        public static final int BL_DRIVE_MOTOR_ID = 5;
        public static final int BL_STEER_MOTOR_ID = 6;
        public static final int BL_STEER_ENCODER_ID = 3;
        public static final double BL_STEER_OFFSET = -Math.toRadians(182.6);

        public static final int BR_DRIVE_MOTOR_ID = 7;
        public static final int BR_STEER_MOTOR_ID = 8;
        public static final int BR_STEER_ENCODER_ID = 4;
        public static final double BR_STEER_OFFSET = -Math.toRadians(38.5);

        public static final double K_MAX_SPEED = 3.0; // 3 meters per second
        public static final double K_MAX_ANGULAR_SPEED = Math.PI; // 1/2 radians rotation per second

        public static final double K_TURN_P = 0.015;
        public static final double K_TURN_I = 0.0;
        public static final double K_TURN_D = 0.0;
        /** This tuning parameter indicates how close to "on target" the PID Controller will attempt to get.*/
        public static final double K_TURN_TOLORANCE_DEGREES = 2.0;
        public static final double K_TURN_TOLORANCE_DEG_PER_SEC = 10;
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
        public static final double kMaxSpeedMetersPerSecond = Drivetrain.MAX_VELOCITY_METERS_PER_SECOND / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond =
                Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}

