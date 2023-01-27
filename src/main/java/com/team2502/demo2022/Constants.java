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
        public static final int CONTROLLER = 3;

        // Buttons

        // Driver Right
        public static final int RESET_HEADING = 2;

        // Driver Left
        public static final int ALLIGN_LEFT = 3;
        public static final int ALLIGN_CENTER = 2;
        public static final int ALLIGN_RIGHT = 4;

        public static final int MOVE_INCHES = 2;
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
            public static final double MAX_VEL = 7; // driver speed gain (m/s)
            public static final double MAX_ROT = 4; // driver rotation gain (rad/s)
                                                   
            // distance between swerve modules (meters)
            public static final double SWERVE_LENGTH = 0.5207; // 20.5 inches
            public static final double SWERVE_WIDTH = 0.6477; // 25.5 inches

            public static final int SWERVE_ENCODER_COUNTS_PER_REV = 4096;
            public static final int FALCON_ENCODER_TICKS_PER_REV = 2048;
            public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75;
            public static final double SWERVE_TURNING_GEAR_RATIO = 12.8;
            public static final double SWERVE_SPARK_ENCODER_COUNTS_TO_DEGREES = 
                SWERVE_ENCODER_COUNTS_PER_REV / 360;
            public static final double SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES = 
                (360) / (FALCON_ENCODER_TICKS_PER_REV * SWERVE_TURNING_GEAR_RATIO);
                    //(FALCON_ENCODER_TICKS_PER_REV * SWERVE_TURNING_GEAR_RATIO) / 360;
            public static final double WHEEL_CIRCUMFERENCE = (4 / 39.37) * Math.PI;
            // CTRE units are encoder ticks per 100ms
            //public static final double SWERVE_METERS_PER_SECOND_TO_CTRE =
            //        (WHEEL_CIRCUMFERENCE * 10) / (SWERVE_DRIVE_GEAR_RATIO * FALCON_ENCODER_TICKS_PER_REV);
            public static final double SWERVE_METERS_PER_SECOND_TO_CTRE =
                ((FALCON_ENCODER_TICKS_PER_REV * SWERVE_DRIVE_GEAR_RATIO/WHEEL_CIRCUMFERENCE) / 10) ;
            public static final double SWERVE_FALCON_TICKS_PER_INCH = (SWERVE_DRIVE_GEAR_RATIO) * (WHEEL_CIRCUMFERENCE * Math.PI);
        }

        public final class Vision {
            public static final String LIMELIGHT_NETWORK_TABLE = "limelight";
        }
    }
}
