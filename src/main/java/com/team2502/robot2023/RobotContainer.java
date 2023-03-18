// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;

import com.team2502.robot2023.commands.DriveCommand;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import com.team2502.robot2023.subsystems.ElevatorSubsystem;
import com.team2502.robot2023.subsystems.IntakeSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.team2502.robot2023.Constants.*;

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

    protected final ElevatorSubsystem ELEVATOR = new ElevatorSubsystem();
    protected final IntakeSubsystem INTAKE = new IntakeSubsystem();

    public RobotContainer() {
        DRIVETRAIN.setDefaultCommand(new DriveCommand(DRIVETRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        JoystickButton ResetHeading = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RESET_HEADING);
        ResetHeading.whenPressed(new InstantCommand(DRIVETRAIN::resetHeading, DRIVETRAIN));

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

        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.setSpeed(0.3), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));
        new JoystickButton(JOYSTICK_OPERATOR, OI.OUTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.setSpeed(-0.3), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));

        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE_OUT)
                .onTrue(new InstantCommand(() -> INTAKE.setPitchSpeed(-0.3), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setPitchSpeed(0.0), INTAKE));
        new JoystickButton(JOYSTICK_OPERATOR, OI.INTAKE_IN)
                .onTrue(new InstantCommand(() -> INTAKE.setPitchSpeed(0.3), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setPitchSpeed(0.0), INTAKE));

        new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.DRIVER_INTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.setSpeed(-0.3), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));
        new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.DRIVER_OUTAKE)
                .onTrue(new InstantCommand(() -> INTAKE.setSpeed(0.3), INTAKE))
                .onFalse(new InstantCommand(() -> INTAKE.setSpeed(0.0), INTAKE));
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