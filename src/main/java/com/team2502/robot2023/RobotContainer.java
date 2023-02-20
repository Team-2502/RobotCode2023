// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;

import com.team2502.robot2023.commands.DriveCommand;
import com.team2502.robot2023.commands.RunConveyorCommand;
import com.team2502.robot2023.commands.RunElevatorCommand;
import com.team2502.robot2023.commands.RunIntakeCommand;
import com.team2502.robot2023.commands.GotoAbsoluteCommand;

import com.team2502.robot2023.subsystems.ConveyorSubsystem;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import com.team2502.robot2023.subsystems.ElevatorSubsystem;
import com.team2502.robot2023.subsystems.IntakeSubsystem;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    protected final XboxController CONTROLLER = new XboxController(Constants.OI.CONTROLLER);

    protected final IntakeSubsystem INTAKE = new IntakeSubsystem();
    protected final ConveyorSubsystem CONVEYOR = new ConveyorSubsystem();
    protected final ElevatorSubsystem ELEVATOR = new ElevatorSubsystem();

    public RobotContainer() {
        DRIVETRAIN.setDefaultCommand(new DriveCommand(DRIVETRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT, CONTROLLER));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        JoystickButton ResetHeading = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RESET_HEADING);
        ResetHeading.onTrue(new InstantCommand(DRIVETRAIN::resetHeading, DRIVETRAIN));

        JoystickButton RunIntake = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RUN_INTAKE);
        RunIntake.whenHeld(new RunIntakeCommand(INTAKE, CONVEYOR, 0.75, 0.5, 0.6));

        JoystickButton RunIntakeBack = new JoystickButton(JOYSTICK_DRIVE_LEFT, Constants.OI.RUN_INTAKE_BACK);
        RunIntakeBack.whenHeld(new RunIntakeCommand(INTAKE, CONVEYOR, -0.75, -0.5, -0.6));

        JoystickButton ElevatorBot = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.ELEVATOR_BOT);
        ElevatorBot.whenPressed(new RunElevatorCommand(ELEVATOR, ElevatorSubsystem.ElevatorPosition.BOTTOM));

        JoystickButton ElevatorMid = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.ELEVATOR_MID);
        ElevatorMid.whenPressed(new RunElevatorCommand(ELEVATOR, ElevatorSubsystem.ElevatorPosition.MIDDLE));

        JoystickButton ElevatorTop = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.ELEVATOR_TOP);
        ElevatorTop.whenPressed(new RunElevatorCommand(ELEVATOR, ElevatorSubsystem.ElevatorPosition.TOP));

        new JoystickButton(JOYSTICK_OPERATOR, 1)
            .whileTrue( new GotoAbsoluteCommand(DRIVETRAIN,
                    new Pose2d(0, 0, new Rotation2d(0))
                    ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new InstantCommand();
    }
}
