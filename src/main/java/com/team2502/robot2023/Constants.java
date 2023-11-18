// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;
import java.util.*;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
        public static final int JOYSTICK_FIGHT = 5;

        // Buttons

        // Driver Right
        public static final int RESET_HEADING = 14; // only use for odo dbg
        public static final int DRIVER_INTAKE = 1;
        public static final int LOWER_INTAKE = 2;
        public static final int ROTATE_ZERO = 3; // teleop field zero
        public static final int QUADRAGRAMENIZE = 4;

        // Driver Left
        public static final int DRIVER_OUTAKE = 1;
        public static final int RAISE_INTAKE = 2;
        public static final int CUBE_GROUND = 3;
        public static final int RET_MODE = 4;
        public static final int NEAREST_SCORE = 3;
        public static final int DRIFT_RESET = 8;

        // DBG
        public static final int DEBUG_RUN = 12;
        public static final int RESET_MODULES = 9;
        public static final int CUBE_LAYER = 1;
        public static final int ELEVATOR_GROUND = 4;
        public static final int ELEVATOR_SINGLE = 2;
        public static final int ELEVATOR_MID = 5;
        public static final int ELEVATOR_TOP = 6;
        public static final int INTAKE_PROTECT = 3;

        // Operator
        public static final int ELEVATOR_STO = 14;
        public static final int ELEVATOR_BOT = 13;

        public static final int ELEVATOR_EXTEND = 9;
        public static final int ELEVATOR_RETRACT = 8;

        public static final int MANIPULATOR_EXTEND = 6;
        public static final int MANIPULATOR_RETRACT = 7;

        public static final int MANIPULATOR_GRAB = 1;
        public static final int MANIPULATOR_RELEASE = 2;


        public static final int RUN_CONVEYOR = 3;
        public static final int RUN_CONVEYOR_BACK = 4;

        public static final int ELEVATOR_OVERRIDE = 5;

        public static final int ARM_EXTEND = 6;
        public static final int ARM_RETRACT = 7;

        public static final int INTAKE = 1;
        public static final int OUTAKE = 2;

        public static final int INTAKE_OUT = 3;
        public static final int INTAKE_IN = 4;

        public static final int MANIPULATOR_HOME = 16;

        public static final int DEFAULT_LED = 13;
        public static final int REQ_CONE = 12;
        public static final int REQ_CUBE = 11;

        public static final int SHELF = 10;

        public static final int OP_CUBE_LAYER = 14;
        public static final int OP_ELEVATOR_GROUND = 13;
        public static final int OP_ELEVATOR_SINGLE = 2;
        public static final int OP_ELEVATOR_MID = 12;
        public static final int OP_ELEVATOR_TOP = 11;
        public static final int OP_INTAKE_PROTECT = 16;
        public static final int OP_SHELF = 15;
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
        public static final class Leds {
            public static final int PORT = 7; // pwm:
            public static final int LED_COUNT = 84; // logical, not physical count (same on 2815)
            public static final int FRAME_RATE = 1;
            public static final int FRAME_TIME = 1/FRAME_RATE; // seconds per frame

            public static final int LED_AHEAD = 0; // Led id corresponding to center of front
            public static final int LED_LEFT = LED_COUNT/4; // Led id corresponding to center of left side

            public static final Color8Bit RED = new Color8Bit(255,0,0);
            public static final Color8Bit BLU = new Color8Bit(0,255,0);
            public static final Color8Bit WHI = new Color8Bit(188,188,188);
        }

        public static final class Arm {
            public static final boolean NT_TUNE = false;

            public static final double ELEVATOR_LIM_TOP = 85;
            public static final double ELEVATOR_LIM_BOTTOM = 0;

            public static final double ELEVATOR_P = 0.5;
            public static final double ELEVATOR_I = 0.0; // TODO: Adjust the integral. This will require retuning the presets. Start at 1.0 and go down.
            public static final double ELEVATOR_D = 0.0;
            public static final double ELEVATOR_MIN_OUTPUT = -0.75; // -1
            public static final double ELEVATOR_MAX_OUTPUT = 0.75; // 1
            public static final double ELEVATOR_MIN_OUTPUT_TELEOP = -0.65;
            public static final double ELEVATOR_MAX_OUTPUT_TELEOP = 0.65;
            public static final double ELEVATOR_THRESHOLD = 1; // rotations until accepted

            public static final double PITCH_P = 0.5;
            public static final double PITCH_I = 0; // TODO: Adjust the integral. This will require retuning the presets. Start at 1.0 and go down.
            public static final double PITCH_D = 0.0;
            public static final double PITCH_MIN_OUTPUT = -0.4; // -1
            public static final double PITCH_MAX_OUTPUT = 0.4; // 1
            public static final double PITCH_MIN_OUTPUT_TELEOP = -0.4; // 0.5
            public static final double PITCH_MAX_OUTPUT_TELEOP = 0.4;
            public static final double PITCH_THRESHOLD = 1;

            public static enum ElevatorPosition {
                BOTTOM(0),
                SLIGHT_EXTEND(13),
                MIDDLE(-26.14), // TODO: measure
                SAFE_PITCH(-29),
                GROUND_PICKUP(-1.69),
                CONE_SINGLE(0),
                CONE_BOTTOM(0),
                CUBE_MID(29),
                CUBE_TOP(70),
                CONE_GROUND_PICKUP(9),
                CONE_MID(60),
                CONE_TOP(87),  
                TOP(-50); // TODO: measure

                public final double position;
                private ElevatorPosition(double position) {
                    this.position = position;
                }
            }

            public static enum ElevatorPitch {
                STOWED(0),
                FRAME_INTERSECT(-25), // TODO: measure on hardware
                GROUND_PICKUP(-41.76),
                CUBE_TOP(-68),
                CONE_TOP(-86.76),
                CONE_MID(-79.76),
                CONE_BOTTOM(-72),
                OUT(-72);

                public final double position;
                private ElevatorPitch(double position) {
                    this.position = position;
                }
            }

            public static final double INTAKE_P = 0.5;
            public static final double INTAKE_I = 0; // TODO: Adjust the integral. This will require retuning the presets. Start at 1.0 and go down.
            public static final double INTAKE_D = 0;
            public static final double INTAKE_MIN_OUTPUT = 0; // 0.2
            public static final double INTAKE_MAX_OUTPUT = 0;

            // These are calculated as integer arithmetic, not floating point
            public static final double ELBOW_ROT_TO_DEGREE = 360 / (48 / 30) / 80; // 80:1 gearbox 2:1 gears - 4 using integer math, 2.8 using floating point
            public static final double WRIST_ROT_TO_DEGREE = 360 / 2 / 16; // 16:1 gearbox 2:1 chain - 11 using integer math, 11.25 using floating point

            // zero degrees is level with floor ahead of robot
            public static final double ELBOW_ZERO_ANGLE = 80;
            public static final double WRIST_ZERO_ANGLE = -ELBOW_ZERO_ANGLE + 255;

            public static enum IntakePosition {
                // elbow, wrist
                // (elbow ang), (wrist ang soli) on NT
                IN(246,169),
                OUT(0,0),
                LEVEL(0,0), 
                CONE_OUT(12,0),
                INIT(ELBOW_ZERO_ANGLE,WRIST_ZERO_ANGLE), // don't use as setpoint
                PORTAL(107, 115),
                SHELF(227, -5),
                CONE_GROUND(157,-36),
                CONE_MID(316,127),
                CONE_TOP(300,131), 
                CONE_SINGLE(146,72), 
                CUBE_GROUND_PICKUP(72,92),
                CUBE_GROUND(100,78),
                CUBE_MID(316,138),
                CUBE_TOP(336,127);

                /*IN(100,180),
                OUT(0,0),
                STOW(27, 171),
                LEVEL(0,0),
                CONE_OUT(12,0),
                INIT(ELBOW_ZERO_ANGLE,WRIST_ZERO_ANGLE), // don't use as setpoint
                PORTAL(-137, 143),
                CONE_GROUND(-75,-14),
                CONE_MID(75,128),
                CONE_TOP(71,135),
                CONE_SINGLE(-104,100),
                CUBE_GROUND(-144,90),
                CUBE_MID(103,172),
                CUBE_TOP(101,166);*/

                public final double pitchElbow;
                public final double pitchWrist;

                private IntakePosition(double elbowAngle, double wristAngle) {
                    this.pitchElbow = -(elbowAngle-ELBOW_ZERO_ANGLE) / ELBOW_ROT_TO_DEGREE;
                    this.pitchWrist = -(wristAngle - WRIST_ZERO_ANGLE) / WRIST_ROT_TO_DEGREE;
                }
            }
        }

        public static final class Intake{
        }

        public static final class Field {
            public static final double FIELD_CENTER_X = 8.294; // TODO: get from cad
            
            public static final AprilTagFieldLayout apriltagPositions;

            //x y z coordinates are in meters, rotations are in radians
            static {
                List<AprilTag> tagList = new ArrayList<>(
                Arrays.asList( // TODO: undo
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
                apriltagPositions = new AprilTagFieldLayout(tagList, 16.54, 8.02);
            }
            //field length, field width are in meters

            static final double COLUMN_GAP = 1; // distance between adjacent posts on the same level // TODO
            static final double ROW_GAP = 1; // distance between levels // TODO

            static final Translation3d CUBE_SOUTH_HIGH = new Translation3d(13.39, 0, 0.9017); // TODO
            static final Translation3d CUBE_SOUTH_MID = new Translation3d(0, 0, 0.6); // TODO
            static final double CUBE_OFFSET = CUBE_SOUTH_HIGH.getZ() - CUBE_SOUTH_MID.getZ();

            static final Translation3d CONE_SOUTH_HIGH = new Translation3d(13.39, 0, 1.1684); // TODO
            static final Translation3d CONE_SOUTH_MID = new Translation3d(0, 0, 0.8636); // TODO
            static final double CONE_OFFSET = CONE_SOUTH_HIGH.getZ() - CONE_SOUTH_MID.getZ();

            public static final ArrayList<Pose2d> scoreLocations;

            static {
                List<Pose2d> scoreLocationsList = new ArrayList<>(
                        Arrays.asList(
                                new Pose2d(1.75, 0.5, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 1.07, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 1.6, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 2.2, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 2.75, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 3.3, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 3.85, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 4.42, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.75, 4.95, Rotation2d.fromDegrees(0))
                        )
                );
                scoreLocations = new ArrayList(scoreLocationsList);
            }

            ///** array of cone post translations, with [0][0] corresponding to the southwest post */
            //public static final Translation3d[][] CONE_GRIDS;
            ///** array of cube post translations, with [0][0] corresponding to the southwest post */
            //public static final Translation3d[][] CUBE_GRIDS;

            //static { // calculate all scoring positions from bottom two grid locations
            //    Translation3d[][] conePosts = new Translation3d[2][6];
            //    Translation3d[][] cubePosts = new Translation3d[2][3];

            //    for (int i = 0; i < 1; i++) { // row
            //        int cones = 0; // not just a modulo
            //        for (int j = 0; i < 8; i++) { // column
            //            if (j%3 == 1) {
            //                cubePosts[i][(int)j/3] = new Translation3d(i*ROW_GAP, j*COLUMN_GAP, i*CUBE_OFFSET).plus(CUBE_SOUTH_MID);
            //            } else {
            //                conePosts[i][cones] = new Translation3d(i*ROW_GAP, j*COLUMN_GAP, i*CONE_OFFSET).plus(CONE_SOUTH_MID);
            //                cones++;
            //            }
            //            
            //        }
            //    }

            //    CONE_GRIDS = conePosts;
            //    CUBE_GRIDS = cubePosts;
            //}
        }
        public static final class PhotonVision {
            public static final String CAMERA_NAME = "USB_2M_GS_camera";
            public static final Transform3d ROBOT_TO_PHOTONVISION = new Transform3d(new Translation3d(-0.35, 0.3, 0.2349), new Rotation3d(0,0, Math.PI)); // position of camera relative to center of robot  TODO: measure accurately
        }
        public static final class Drivetrain {
            public static final double MAX_VEL = 7; // driver speed gain (m/s) 11 - 7
            public static final double MAX_ROT = 3; // driver rotation gain (rad/s) 9 - 4
            public static final double RET_VEL = 1; // driver speed gain (m/s)
            public static final double RET_ROT = 0.25; // driver rotation gain (rad/s)
            public static final double OI_DEADZONE_XY = 0.02; // Joystick deadzone 
            public static final double OI_DEADZONE_Z = 0.03; // Joystick twist deadzone 
            public static final double DEADZONE_XY = 0.05; // Transpose deadzone
            public static final double DEADZONE_THETA = 0.05; // Rotation deadzone
                                                
            // constants for pose control
            public static final double DRIVETRAIN_MOVE_P = 2.7; // 0.6
            public static final double DRIVETRAIN_MOVE_I = 0.0003;
            public static final double DRIVETRAIN_MOVE_D = 0.25;
            public static final double DRIVETRAIN_MOVE_A = 3;
            public static final double DRIVETRAIN_TURN_P = 2;
            public static final double DRIVETRAIN_TURN_I = 0.001;
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
