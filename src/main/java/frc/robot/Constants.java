// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

        public static final int DRIVETRAIN_PIGEON_ID = 0;

        public static final int FL_DRIVE_MOTOR_ID = 1;
        public static final int FL_STEER_MOTOR_ID = 2;
        public static final int FL_STEER_ENCODER_ID = 1;
        public static final double FL_STEER_OFFSET = -Math.toRadians(75.67);

        public static final int FR_DRIVE_MOTOR_ID = 3;
        public static final int FR_STEER_MOTOR_ID = 4;
        public static final int FR_STEER_ENCODER_ID = 2;
        public static final double FR_STEER_OFFSET = -Math.toRadians(258.83);

        public static final int BR_DRIVE_MOTOR_ID = 5;
        public static final int BR_STEER_MOTOR_ID = 6;
        public static final int BR_STEER_ENCODER_ID = 3;
        public static final double BR_STEER_OFFSET = -Math.toRadians(110.21);

        public static final int BL_DRIVE_MOTOR_ID = 7;
        public static final int BL_STEER_MOTOR_ID = 8;
        public static final int BL_STEER_ENCODER_ID = 4;
        public static final double BL_STEER_OFFSET = -Math.toRadians(63.54);


        public static final double K_MAX_SPEED = 3.0; // 3 meters per second
        public static final double K_MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        }
    public final class PS4 {
        private PS4() {
        }
        public static final int CONTROLLER_ID = 1;
        /**Left Stick Tilt Left/Right*/
        public static final int X_AXES = 1; 
        /**Left Stick Tilt Forwards/Backwards*/
        public static final int Y_AXES = 0;
        /**Right Stick Tilt Left/Right*/
        public static final int Z_AXES = 2;
        public static final int SLIDER_AXES = 3;
        public static final int TRIGGER_BUTTON = 0;
    }
}

