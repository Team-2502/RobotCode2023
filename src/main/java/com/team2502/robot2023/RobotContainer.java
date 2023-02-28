// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;

import com.team2502.robot2023.Constants.OI;
import com.team2502.robot2023.autonomous.AutoChooser;
import com.team2502.robot2023.commands.*;

import com.team2502.robot2023.subsystems.*;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    protected final Joystick JOYSTICK_OPERATOR = new Joystick(Constants.OI.JOYSTICK_OPERATOR);
    protected final Joystick JOYSTICK_DEBUG = new Joystick(Constants.OI.JOYSTICK_DEBUG);

    protected final XboxController CONTROLLER = new XboxController(Constants.OI.CONTROLLER);

    protected final IntakeSubsystem INTAKE = new IntakeSubsystem();
    protected final ConveyorSubsystem CONVEYOR = new ConveyorSubsystem();
    protected final ElevatorSubsystem ELEVATOR = new ElevatorSubsystem();
    protected final ManipulatorSubsystem MANIPULATOR = new ManipulatorSubsystem();

    public RobotContainer() {
        DRIVETRAIN.setDefaultCommand(new DriveCommand(DRIVETRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT, CONTROLLER));

        AutoChooser.putToSmartDashboard();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        JoystickButton ResetHeading = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RESET_HEADING);
        ResetHeading.onTrue(new InstantCommand(DRIVETRAIN::resetOffset, DRIVETRAIN));

        new JoystickButton(JOYSTICK_DEBUG, OI.RESET_MODULES)
            .onTrue(new InstantCommand(DRIVETRAIN::setSwerveInit, DRIVETRAIN));

        JoystickButton RunIntake = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RUN_INTAKE);
        RunIntake.whileTrue(new RunIntakeCommand(INTAKE, CONVEYOR, 0.5, 0.55, 0.4));

        JoystickButton RunIntakeBack = new JoystickButton(JOYSTICK_DRIVE_LEFT, Constants.OI.RUN_INTAKE_BACK);
        RunIntakeBack.whileTrue(new RunIntakeCommand(INTAKE, CONVEYOR, -0.5, -0.55, -0.4));

        JoystickButton ElevatorBot = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.ELEVATOR_BOT);
        ElevatorBot.onTrue(new SetElevatorCommand(ELEVATOR, Constants.Subsystems.Elevator.ElevatorPosition.BOTTOM));

        JoystickButton ElevatorMid = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.ELEVATOR_MID);
        ElevatorMid.onTrue(new SetElevatorCommand(ELEVATOR, Constants.Subsystems.Elevator.ElevatorPosition.MIDDLE));

        JoystickButton ElevatorTop = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.ELEVATOR_TOP);
        ElevatorTop.onTrue(new SetElevatorCommand(ELEVATOR, Constants.Subsystems.Elevator.ElevatorPosition.TOP));

        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_OUT)
            .onTrue(new InstantCommand(() -> ELEVATOR.setPitch(Constants.Subsystems.Elevator.ElevatorPitch.OUT), ELEVATOR));
        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_IN)
            .onTrue(new InstantCommand(() -> ELEVATOR.setPitch(Constants.Subsystems.Elevator.ElevatorPitch.STOWED), ELEVATOR));

        new JoystickButton(JOYSTICK_DEBUG, OI.DEBUG_RUN)
        //    .whileTrue( new GotoAbsoluteCommand(DRIVETRAIN, new Pose2d(0, 0, new Rotation2d(0))));
            .whileTrue( new FollowPathAbsoluteCommand(DRIVETRAIN, "testpath"));

        new JoystickButton(JOYSTICK_DEBUG, OI.DEBUG_RUN+1)
                .onTrue(new InstantCommand(() -> DRIVETRAIN.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), DRIVETRAIN));

        new JoystickButton(JOYSTICK_DEBUG, 5)
            .whileTrue( new FollowPathAbsoluteCommand(DRIVETRAIN, "../pathplanner/generatedJSON/test-curve"));

        new JoystickButton(JOYSTICK_DEBUG, 6)
            .whileTrue( new BalanceCommand(DRIVETRAIN, false));

        new JoystickButton(JOYSTICK_OPERATOR, OI.ELEVATOR_EXTEND)
                .onTrue(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.3), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.0), ELEVATOR));
        new JoystickButton(JOYSTICK_OPERATOR, OI.ELEVATOR_RETRACT)
                .onTrue(new InstantCommand(() -> ELEVATOR.setLinearSpeed(-0.3), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.0), ELEVATOR));

        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_EXTEND)
                .onTrue(new InstantCommand(() -> ELEVATOR.setPitchSpeed(0.5), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setPitchSpeed(0.0), ELEVATOR));
        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_RETRACT)
                .onTrue(new InstantCommand(() -> ELEVATOR.setPitchSpeed(-0.5), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setPitchSpeed(0.0), ELEVATOR));

        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_GRAB)
                .onTrue(new InstantCommand(() -> MANIPULATOR.setSpeed(0.6)))
                .onFalse(new InstantCommand(() -> MANIPULATOR.setSpeed(0.0)));
        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_RELEASE)
                .onTrue(new InstantCommand(() -> MANIPULATOR.setSpeed(-0.6)))
                .onFalse(new InstantCommand(() -> MANIPULATOR.setSpeed(0.0)));

        new JoystickButton(JOYSTICK_OPERATOR, OI.ELEVATOR_ZERO)
                .onTrue(new InstantCommand(() -> ELEVATOR.set(Constants.Subsystems.Elevator.ElevatorPosition.BOTTOM), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.stop()));

        new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.LOWER_INTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.runLift(0.2)))
                .onFalse(new InstantCommand(() -> INTAKE.runLift(0.0)));

        new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.RAISE_INTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.runLift(-0.2)))
                .onFalse(new InstantCommand(() -> INTAKE.runLift(0.0)));

        new JoystickButton(JOYSTICK_OPERATOR, OI.RUN_CONVEYOR)
                .onTrue(new InstantCommand(() -> CONVEYOR.run(0.4)))
                .onFalse(new InstantCommand(() -> CONVEYOR.run(0.0)));
        new JoystickButton(JOYSTICK_OPERATOR, OI.RUN_CONVEYOR_BACK)
                .onTrue(new InstantCommand(() -> CONVEYOR.run(-0.4)))
                .onFalse(new InstantCommand(() -> CONVEYOR.run(0.0)));

        // Home commands
        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE_HOME)
                .onTrue(new InstantCommand(() -> INTAKE.home(), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.stopLift(), INTAKE));

        new JoystickButton(JOYSTICK_OPERATOR, OI.ELEVATOR_HOME)
                .onTrue(new InstantCommand(() -> ELEVATOR.home(), ELEVATOR))
                .onFalse(new InstantCommand(() -> ELEVATOR.setLinearSpeed(0.0), ELEVATOR));

        new JoystickButton(JOYSTICK_OPERATOR, OI.MANIPULATOR_HOME)
                .onTrue(new InstantCommand(() -> MANIPULATOR.home(), MANIPULATOR))
                .onFalse(new InstantCommand(() -> MANIPULATOR.setSpeed(0.0), MANIPULATOR));
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
                ELEVATOR,
                MANIPULATOR
        );
    }
}
