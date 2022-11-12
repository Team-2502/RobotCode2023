// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.demo2022;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class OI {
        public static final int JOYSTICK_DRIVE_RIGHT = 0;
        public static final int JOYSTICK_DRIVE_LEFT = 1;

        // Buttons

        // Driver Right
        public static final int RESET_GYRO = 2;
    }

    public final class HardwareMap {
        // Front Left Module
        public static final int FL_DRIVE_MOTOR = 1;
        public static final int FL_TURN_MOTOR = 2;
        public static final int FL_TURN_ENCODER = 3;

        // Front Right Module
        public static final int FR_DRIVE_MOTOR = 4;
        public static final int FR_TURN_MOTOR = 5;
        public static final int FR_TURN_ENCODER = 6;

        // Back Left Module
        public static final int BL_DRIVE_MOTOR = 7;
        public static final int BL_TURN_MOTOR = 8;
        public static final int BL_TURN_ENCODER = 9;

        // Back Right Module
        public static final int BR_DRIVE_MOTOR = 10;
        public static final int BR_TURN_MOTOR = 11;
        public static final int BR_TURN_ENCODER = 12;
    }

    public final class Subsystems {
        public final class Drivetrain {
            // distance between swerve modules (meters)
            public static final double SWERVE_LENGTH = 0.762;
            public static final double SWERVE_WIDTH = 0.752;
        }
    }
}
