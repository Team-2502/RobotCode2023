// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;

import com.team2502.robot2023.Constants.OI;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPosition;
import com.team2502.robot2023.Constants.Subsystems.Arm.IntakePosition;
import com.team2502.robot2023.autonomous.AutoChooser;
import com.team2502.robot2023.commands.*;
import com.team2502.robot2023.commands.YawLockedTranspose.Mode;
import com.team2502.robot2023.subsystems.*;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import com.team2502.robot2023.subsystems.ArmSubsystem;
import com.team2502.robot2023.subsystems.IntakeSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.team2502.robot2023.Constants.*;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    protected final DrivetrainSubsystem DRIVETRAIN = new DrivetrainSubsystem();

    protected final Joystick JOYSTICK_DRIVE_LEFT = new Joystick(Constants.OI.JOYSTICK_DRIVE_LEFT);
    protected final Joystick JOYSTICK_DRIVE_RIGHT = new Joystick(Constants.OI.JOYSTICK_DRIVE_RIGHT);
    protected final Joystick JOYSTICK_OPERATOR = new Joystick(OI.JOYSTICK_OPERATOR);
    protected final Joystick JOYSTICK_FIGHT = new Joystick(Constants.OI.JOYSTICK_FIGHT);

    protected final ArmSubsystem ELEVATOR = new ArmSubsystem();
    protected final IntakeSubsystem INTAKE = new IntakeSubsystem();

    protected final LightstripSubsystem LIGHTSTRIP = new LightstripSubsystem(ELEVATOR);

    // protected final PhotonVisionSubsystem VISION = new PhotonVisionSubsystem(DRIVETRAIN);

    public RobotContainer() {
        DRIVETRAIN.setDefaultCommand(new DriveCommand(DRIVETRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT));
        //LIGHTSTRIP.setDefaultCommand(new RunAnimationCommand(LIGHTSTRIP, LightstripSubsystem.Animations.orbit_demo_simple, 1));

        AutoChooser.putToSmartDashboard();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        SmartDashboard.putData("RP pose b", new InstantCommand(() -> DRIVETRAIN.setPose(new Pose2d(1.9, 4.45, Rotation2d.fromDegrees(0)))));
        SmartDashboard.putData("B reset pose mid", new InstantCommand(() -> DRIVETRAIN.setPose(new Pose2d(2, 2.75, Rotation2d.fromDegrees(0))), DRIVETRAIN));

        SmartDashboard.putData("test balance", new BalanceCommand(DRIVETRAIN, false));

        JoystickButton ResetHeading = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RESET_HEADING);
        ResetHeading.whenPressed(new InstantCommand(DRIVETRAIN::resetHeading, DRIVETRAIN));

        new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.ROTATE_ZERO)
            .whenPressed(new InstantCommand(DRIVETRAIN::resetOffset, DRIVETRAIN));

        //new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.NEAREST_SCORE)
        //        .whileTrue(new GotoNearestScoreCommand(DRIVETRAIN, VISION));

        //new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.ROTATE_ZERO)
        //        .whileTrue()

        //new JoystickButton(JOYSTICK_DRIVE_LEFT, 8).whileTrue(new RunAnimationCommand(LIGHTSTRIP, LightstripSubsystem.Animations.orbit_demo, 1));

        new JoystickButton(JOYSTICK_OPERATOR, OI.ELEVATOR_EXTEND)
                .onTrue(new InstantCommand(() -> ELEVATOR.setLinearSpeed(-0.5), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.0), ELEVATOR));
        new JoystickButton(JOYSTICK_OPERATOR, OI.ELEVATOR_RETRACT)
                .onTrue(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.5), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.0), ELEVATOR));

        new JoystickButton(JOYSTICK_OPERATOR, OI.ARM_EXTEND)
                .onTrue(new InstantCommand(() -> ELEVATOR.setPitchSpeed(-0.3), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setPitchSpeed(0.0), ELEVATOR));
        new JoystickButton(JOYSTICK_OPERATOR, OI.ARM_RETRACT)
                .onTrue(new InstantCommand(() -> ELEVATOR.setPitchSpeed(0.3), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setPitchSpeed(0.0), ELEVATOR));

        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE_OUT)
                .onTrue(new InstantCommand(() -> ELEVATOR.setArmPitchSpeed(-0.3), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setArmPitchSpeed(0.0), ELEVATOR));
        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE_IN)
                .onTrue(new InstantCommand(() -> ELEVATOR.setArmPitchSpeed(0.3), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setArmPitchSpeed(0.0), ELEVATOR));

        new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.DRIVER_INTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.setSpeed(-0.7), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));
        new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.DRIVER_OUTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.setSpeed(0.7), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));

        //new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.CUBE_GROUND)
        //        .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND));

        //new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.CUBE_GROUND+1)
        //        .whileTrue(new AutoPickupCommand(DRIVETRAIN, ELEVATOR, true));

        /*new JoystickButton(JOYSTICK_OPERATOR, OI.DEFAULT_LED)
                .onTrue(new RunAnimationCommand(LIGHTSTRIP, LightstripSubsystem.Animations.disabled, 1));
        new JoystickButton(JOYSTICK_OPERATOR, OI.REQ_CONE)
                .onTrue(new RunAnimationCommand(LIGHTSTRIP, LightstripSubsystem.Animations.request_cone, 1));
        new JoystickButton(JOYSTICK_OPERATOR, OI.REQ_CUBE)
                .onTrue(new RunAnimationCommand(LIGHTSTRIP, LightstripSubsystem.Animations.request_cube, 1));
*/
        Trigger cubeButton = new JoystickButton(JOYSTICK_FIGHT, OI.CUBE_LAYER).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_CUBE_LAYER));

		// cube positions
        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_GROUND).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_ELEVATOR_GROUND))
            .and(cubeButton)
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND_PICKUP));

        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_MID).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_ELEVATOR_MID))
            .and(cubeButton)
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.CUBE_MID, IntakePosition.CUBE_MID));

        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_TOP).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_ELEVATOR_TOP))
            .and(cubeButton)
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.CUBE_TOP, IntakePosition.CUBE_TOP));

		// cone positions
        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_GROUND).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_ELEVATOR_GROUND))
            .and(cubeButton.negate())
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.BOTTOM, IntakePosition.CONE_GROUND));

        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_SINGLE).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_SHELF))
            .and(cubeButton.negate())
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.BOTTOM, IntakePosition.PORTAL));

        new JoystickButton(JOYSTICK_OPERATOR, OI.SHELF)
                .and(cubeButton.negate())
                .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.BOTTOM, IntakePosition.SHELF));

        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_MID).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_ELEVATOR_MID))
            .and(cubeButton.negate())
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.CONE_MID, IntakePosition.CONE_MID));

        new JoystickButton(JOYSTICK_FIGHT, OI.ELEVATOR_TOP).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_ELEVATOR_TOP))
            .and(cubeButton.negate())
            .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.CONE_TOP, IntakePosition.CONE_TOP));

		// cube style intake
        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE)
            .and(cubeButton)
			.onTrue(new InstantCommand(() -> INTAKE.setSpeed(0.7), INTAKE))
			.onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));
        new JoystickButton(JOYSTICK_OPERATOR, OI.OUTAKE)
            .and(cubeButton)
			.onTrue(new InstantCommand(() -> INTAKE.setSpeed(-0.7), INTAKE))
			.onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));

		// cone style intake
        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE)
            .and(cubeButton.negate())
			.onTrue(new InstantCommand(() -> INTAKE.setSpeed(-0.7), INTAKE))
			.onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));
        new JoystickButton(JOYSTICK_OPERATOR, OI.OUTAKE)
            .and(cubeButton.negate())
			.onTrue(new InstantCommand(() -> INTAKE.setSpeed(0.7), INTAKE))
			.onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));

        // stow
        new JoystickButton(JOYSTICK_FIGHT, OI.INTAKE_PROTECT).or(new JoystickButton(JOYSTICK_OPERATOR, OI.OP_INTAKE_PROTECT))
                .whileTrue(new SetArmSimpleCommand(ELEVATOR, ElevatorPosition.BOTTOM, IntakePosition.IN));

        new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.QUADRAGRAMENIZE)
                .whileTrue(new GotoNearest90Command(DRIVETRAIN));

		// Debug
        new JoystickButton(JOYSTICK_FIGHT, OI.DEBUG_RUN)
        //  .whileTrue( new YawLockedTranspose(DRIVETRAIN, new ChassisSpeeds(-1,0,0)));
			.whileTrue(new BalanceCommand(DRIVETRAIN, false));
	    //	.whileTrue( new FollowPathAbsoluteCommand(DRIVETRAIN, "../pathplanner/generatedJSON/forward-turn"));
        //    .onTrue(Commands.sequence(
        //        new InstantCommand(DRIVETRAIN::resetHeading),
        //        Commands.deadline(Commands.waitSeconds(0.37), new YawLockedTranspose(DRIVETRAIN, new ChassisSpeeds(-.8,0,0), Mode.NAVX_ZERO)),
        //        Commands.deadline(Commands.waitSeconds(1.5), new YawLockedTranspose(DRIVETRAIN, new ChassisSpeeds(-1,0,0), Mode.NAVX_ZERO)),
        //        Commands.deadline(Commands.waitSeconds(0.5), new YawLockedTranspose(DRIVETRAIN, new ChassisSpeeds(-0.9,0,0), Mode.NAVX_ZERO)),
        //        Commands.waitSeconds(0.5),
        //        Commands.deadline(Commands.waitSeconds(1.2), new YawLockedTranspose(DRIVETRAIN, new ChassisSpeeds(1.0,0,0), Mode.NAVX_ZERO)),
        //        Commands.deadline(new TimeLeftCommand(0.75), new BalanceCommand(DRIVETRAIN, false)),
        //        Commands.deadline(Commands.waitSeconds(0.25), new YawLockedTranspose(DRIVETRAIN, new ChassisSpeeds(0,-.3,0), Mode.NAVX_ZERO))
        //                ));
        //new NetworkButton(NetworkTableInstance.getDefault().getBooleanTopic("testbtn"));

        //new JoystickButton(JOYSTICK_FIGHT, OI.DEBUG_RUN)
        //    .onTrue(new InstantCommand(() -> DRIVETRAIN.resetHeading()))
        //    .onTrue(new InstantCommand(() -> DRIVETRAIN.setPose(new Pose2d(1.75, 4.45, Rotation2d.fromDegrees(0))), DRIVETRAIN))
        //    .whileTrue( new FollowPathAbsoluteCommand(DRIVETRAIN, "../pathplanner/generatedJSON/alliancetest"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoChooser.getAutoInstance().getInstance(
                DRIVETRAIN,
                INTAKE,
                ELEVATOR
        );
    }
}
