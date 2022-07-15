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

    public final class Drivetrain {
        private Drivetrain() {
        }

        public static final int FL_DRIVE_MOTOR_ID = 1;
        public static final int FR_DRIVE_MOTOR_ID = 3;
        public static final int BR_DRIVE_MOTOR_ID = 5;
        public static final int BL_DRIVE_MOTOR_ID = 7;
        public static final int FL_TURN_MOTOR_ID = 2;
        public static final int FR_TURN_MOTOR_ID = 4;
        public static final int BR_TURN_MOTOR_ID = 6;
        public static final int BL_TURN_MOTOR_ID = 8;

        public static final int GYRO_ID = 1;

        public static final double K_MAX_SPEED = 3.0; // 3 meters per second
        public static final double K_MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        }
    public final class PS4 {
        private PS4() {
        }
        public static final int CONTROLLER_ID = 1;
        /**Left Stick Tilt Left/Right*/
        public static final int X_AXES = 0; 
        /**Left Stick Tilt Forwards/Backwards*/
        public static final int Y_AXES = 1;
        /**Right Stick Tilt Left/Right*/
        public static final int Z_AXES = 2;
        public static final int SLIDER_AXES = 3;
        public static final int TRIGGER_BUTTON = 0;
    }
}

