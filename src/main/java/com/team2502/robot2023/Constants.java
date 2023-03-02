// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;
import java.util.*;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
        public static final int JOYSTICK_OPERATOR = 2;
        public static final int JOYSTICK_DEBUG = 5;
        public static final int CONTROLLER = 3;

        // Buttons

        // Driver Right
        public static final int RESET_HEADING = 3;
        public static final int RUN_INTAKE = 1;
        public static final int LOWER_INTAKE = 2;
        public static final int RET_MODE = 4;

        // Driver Left
        public static final int RUN_INTAKE_BACK = 1;
        public static final int RAISE_INTAKE = 2;


        // DBG
        public static final int DEBUG_RUN = 1;
        public static final int RESET_MODULES = 4;

        // Operator
        public static final int ELEVATOR_BOT = 12;
        public static final int ELEVATOR_MID = 11;
        public static final int ELEVATOR_TOP = 15;

        public static final int MANIPULATOR_OUT = 14;
        public static final int MANIPULATOR_IN = 13;

        public static final int ELEVATOR_EXTEND = 9;
        public static final int ELEVATOR_RETRACT = 8;

        public static final int MANIPULATOR_EXTEND = 6;
        public static final int MANIPULATOR_RETRACT = 7;

        public static final int MANIPULATOR_GRAB = 1;
        public static final int MANIPULATOR_RELEASE = 2;

        public static final int ELEVATOR_ZERO = 16;

        public static final int RUN_CONVEYOR = 3;
        public static final int RUN_CONVEYOR_BACK = 4;

        // Home commands
        public static final int INTAKE_HOME = 14;
        public static final int ELEVATOR_HOME = 15;
        public static final int MANIPULATOR_HOME = 16;
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
        public static final int LEFT_LIFT_INTAKE_MOTOR = 14;
        public static final int RIGHT_LIFT_INTAKE_MOTOR = 21;
        public static final int RIGHT_INTAKE_MOTOR = 15;
        public static final int LEFT_INTAKE_MOTOR = 16;
        public static final int SWITCH_LEFT_INTAKE = 0;
        public static final int SWITCH_RIGHT_INTAKE = 1;

        // Elevator
        public static final int LEFT_ELEVATOR_MOTOR = 17;
        public static final int RIGHT_ELEVATOR_MOTOR = 18;
        public static final int PITCH_ELEVATOR_MOTOR = 19;
        public static final int SWITCH_ELEVATOR = 2;

        // Manipulator
        public static final int GRIPPER_MOTOR = 20;
        public static final int SWITCH_GRIPPER = 7;

    }

    public static final class Subsystems {
        public static final class Elevator {
            public static final boolean NT_TUNE = false;

            public static final double ELEVATOR_LIM_TOP = -50;
            public static final double ELEVATOR_LIM_BOTTOM = 0;

            public static final double ELEVATOR_P = 0.5;
            public static final double ELEVATOR_I = 0.0;
            public static final double ELEVATOR_D = 0.0;
            public static final int ELEVATOR_MIN_OUTPUT = -1;
            public static final int ELEVATOR_MAX_OUTPUT = 1;

            public static final double PITCH_P = 0.5;
            public static final double PITCH_I = 0.0;
            public static final double PITCH_D = 0.0;
            public static final int PITCH_MIN_OUTPUT = -1;
            public static final int PITCH_MAX_OUTPUT = 1;

            public static enum ElevatorPosition {
                BOTTOM(0),
                MIDDLE(1),
                SAFE_PITCH(-29),
                CUBE_TOP(-48.5),
                TOP(-47.9);

                public final double position;
                private ElevatorPosition(double position) {
                    this.position = position;
                }
            }

            public static enum ElevatorPitch {
                STOWED(0),
                CUBE_TOP(-68),
                OUT(-72);

                public final double position;
                private ElevatorPitch(double position) {
                    this.position = position;
                }
            }
        }

        public static final class Manipulator {
            public static final double GRIPPER_P = 0.5;
            public static final double GRIPPER_I = 0.0;
            public static final double GRIPPER_D = 0.0;

            public static enum ManipulatorPosition {
                OPEN(8),
                STOWED(41), // does not intersect stowed intake
                CONE(115), // TODO : measure
                CUBE(81),  // TODO : measure 
                CLOSED(160);

                public final double position;
                private ManipulatorPosition(double position) {
                    this.position = position;
                }
            }

            
        }

        public static final class Intake {
            public static enum IntakePosition {
                DEPLOYED(0),
                RETRACTED(1);

                public final double position;
                private IntakePosition(double position) {
                    this.position = position;
                }
            }
        }

        public static final class AprilTags {
            public static final double FIELD_CENTER_X = 8.294; // TODO: get from cad
            
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
            public static final Transform3d ROBOT_TO_PHOTONVISION = new Transform3d(new Translation3d(0.2, 0.0, 0.66), new Rotation3d(0,0, Math.PI)); // position of camera relative to center of robot  TODO: measure accurately
        }
        public static final class Drivetrain {
            public static final double MAX_VEL = 6; // driver speed gain (m/s)
            public static final double MAX_ROT = 3; // driver rotation gain (rad/s)
            public static final double RET_VEL = 6; // driver speed gain (m/s)
            public static final double RET_ROT = 3; // driver rotation gain (rad/s)
                                                
            // constants for pose control
            public static final double DRIVETRAIN_MOVE_P = 1.4;
            public static final double DRIVETRAIN_MOVE_I = 0.0003;
            public static final double DRIVETRAIN_MOVE_D = 0.0;
            public static final double DRIVETRAIN_MOVE_A = 3;
            public static final double DRIVETRAIN_TURN_P = 1.6;
            public static final double DRIVETRAIN_TURN_I = 0.0007;
            public static final double DRIVETRAIN_TURN_D = 0;
            public static final double DRIVETRAIN_TURN_A = 2;
                                                   
            // distance between swerve modules (meters)
            public static final double SWERVE_LENGTH = 0.5405;
            public static final double SWERVE_WIDTH = 0.5405;

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
