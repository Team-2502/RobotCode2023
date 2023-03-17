// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;
import java.util.*;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OI {
        public static final int JOYSTICK_DRIVE_RIGHT = 0;
        public static final int JOYSTICK_DRIVE_LEFT = 1;
        public static final int CONTROLLER = 3;

        // Buttons

        // Driver Right
        public static final int RESET_HEADING = 2;
        public static final int DRIVER_INTAKE = 1;

        // Driver Left
        public static final int DRIVER_OUTAKE = 1;

        // Operator
        public static final int ELEVATOR_EXTEND = 9;
        public static final int ELEVATOR_RETRACT = 8;

        public static final int ARM_EXTEND = 6;
        public static final int ARM_RETRACT = 7;

        public static final int INTAKE = 1;
        public static final int OUTAKE = 2;

        public static final int INTAKE_OUT = 3;
        public static final int INTAKE_IN = 4;
    }

    public static final class HardwareMap {
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

        // Conveyor Belt
        public static final int CONVEYOR = 13;

        // Intake
        public static final int INTAKE = 14;
        public static final int INTAKE_PITCH = 15;
        public static final int SWITCH_INTAKE = 3;

        // Elevator
        public static final int LEFT_ELEVATOR_MOTOR = 17;
        public static final int RIGHT_ELEVATOR_MOTOR = 18;
        public static final int LEFT_PITCH_ELEVATOR_MOTOR = 19;
        public static final int RIGHT_PITCH_ELEVATOR_MOTOR = 20;
        public static final int SWITCH_ELEVATOR = 2;

    }

    public static final class Subsystems {
        public static final class Intake {

            public static final int INTAKE_P = 0;
            public static final int INTAKE_I = 0;
            public static final int INTAKE_D = 0;

            public static enum IntakePosition {
                IN(0), OUT(0), LEVEL(0); // TODO: measure

                public final double position;

                private IntakePosition(double position) {
                    this.position = position;
                }
            }
        }

        public static final class Elevator {
            public static final boolean NT_TUNE = false;

            public static final double ELEVATOR_LIM_TOP = -50;
            public static final double ELEVATOR_LIM_BOTTOM = 0;

            public static final double ELEVATOR_P = 0.5;
            public static final double ELEVATOR_I = 0.0;
            public static final double ELEVATOR_D = 0.0;
            public static final double ELEVATOR_MIN_OUTPUT = -1;
            public static final double ELEVATOR_MAX_OUTPUT = 1;
            public static final double ELEVATOR_MIN_OUTPUT_TELEOP = -0.65;
            public static final double ELEVATOR_MAX_OUTPUT_TELEOP = 0.65;
            public static final double ELEVATOR_THRESHOLD = 1; // rotations until accepted

            public static final double PITCH_P = 0.5;
            public static final double PITCH_I = 0.0;
            public static final double PITCH_D = 0.0;
            public static final double PITCH_MIN_OUTPUT = -1;
            public static final double PITCH_MAX_OUTPUT = 1;
            public static final double PITCH_MIN_OUTPUT_TELEOP = -0.5;
            public static final double PITCH_MAX_OUTPUT_TELEOP = 0.5;
            public static final double PITCH_THRESHOLD = 1;

            public static enum ElevatorPosition {
                BOTTOM(0), MIDDLE(-26.14), // TODO: measure
                SAFE_PITCH(-29), GROUND_PICKUP(-1.69), CUBE_TOP(-48.5), CONE_BOTTOM(0), CONE_TOP(-49.7), TOP(-50); // TODO: measure

                public final double position;

                private ElevatorPosition(double position) {
                    this.position = position;
                }
            }

            public static enum ElevatorPitch {
                STOWED(0), FRAME_INTERSECT(-25), // TODO: measure on hardware
                GROUND_PICKUP(-41.76), CUBE_TOP(-68), CONE_TOP(-86.76), CONE_MID(-79.76), CONE_BOTTOM(-72), OUT(-72);

                public final double position;

                private ElevatorPitch(double position) {
                    this.position = position;
                }
            }
        }

        public static final class AprilTags {
            
            public static final AprilTagFieldLayout field;

            //x y z coordinates are in meters, rotations are in radians
            static {
                List<AprilTag> tagList = new ArrayList<>(
                Arrays.asList(
                    new AprilTag(1, new Pose3d(15.51358903, 1.071628143, 0.462788926, new Rotation3d(0,0, Math.PI))),
                    new AprilTag(2, new Pose3d(15.51358903, 2.748031496, 0.462788926, new Rotation3d(0,0, Math.PI))),
                    new AprilTag(3, new Pose3d(15.51358903, 4.424434849, 0.462788926, new Rotation3d(0,0, Math.PI))),
                    new AprilTag(4, new Pose3d(16.17881636, 6.7498095, 0.695453391, new Rotation3d(0,0, Math.PI))),
                    new AprilTag(5, new Pose3d(0.361950724, 6.7498095, 0.695453391, new Rotation3d(0,0, 0))),
                    new AprilTag(6, new Pose3d(1.027432055, 4.424434849, 0.462788926, new Rotation3d(0,0, 0))),
                    new AprilTag(7, new Pose3d(1.027432055, 2.748031496, 0.462788926, new Rotation3d(0,0, 0))),
                    new AprilTag(8, new Pose3d(1.027432055, 1.071628143, 0.462788926, new Rotation3d(0,0, 0)))
                    )
                );
                field = new AprilTagFieldLayout(tagList, 16.54, 8.02);
            }
            //field length, field width are in meters
        }
        public static final class PhotonVision {
            public static final String CAMERA_NAME = "HD_Pro_Webcam_C920";
        }
        public static final class Drivetrain {
            public static final double MAX_VEL = 7; // driver speed gain (m/s)
            public static final double MAX_ROT = 4; // driver rotation gain (rad/s)
                                                   
            // distance between swerve modules (meters)
            public static final double SWERVE_LENGTH = 0.5334; // TODO: get from cad
            public static final double SWERVE_WIDTH = 0.5334;

            public static final int SWERVE_ENCODER_COUNTS_PER_REV = 4096;
            public static final int FALCON_ENCODER_TICKS_PER_REV = 2048;
            public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75;
            public static final double SWERVE_TURNING_GEAR_RATIO = 15.43;
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
            public static final double SWERVE_FALCON_TICKS_PER_INCH = (SWERVE_DRIVE_GEAR_RATIO) * (WHEEL_CIRCUMFERENCE * Math.PI); // actually in meters, haha
            public static final double SWERVE_FALCON_METERS_PER_TICK = WHEEL_CIRCUMFERENCE / (FALCON_ENCODER_TICKS_PER_REV*SWERVE_DRIVE_GEAR_RATIO);
        }


        public static final class Vision {
            public static final String LIMELIGHT_NETWORK_TABLE = "limelight";
            //TODO SET CORRECT NUMBERS
            public static final double LIMELIGHT_ON = 0;
            public static final double LIMELIGHT_OFF = 1;
        }
    }
}
