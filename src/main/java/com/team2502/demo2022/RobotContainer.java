// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.demo2022;

import com.team2502.demo2022.commands.DriveCommand;
import com.team2502.demo2022.commands.allignApriltagCommand;
import com.team2502.demo2022.subsystems.DrivetrainSubsystem;
import com.team2502.demo2022.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Joystick;
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
    protected final VisionSubsystem VISION = new VisionSubsystem();

    protected final Joystick JOYSTICK_DRIVE_LEFT = new Joystick(Constants.OI.JOYSTICK_DRIVE_LEFT);
    protected final Joystick JOYSTICK_DRIVE_RIGHT = new Joystick(Constants.OI.JOYSTICK_DRIVE_RIGHT);

    public RobotContainer() {
        DRIVETRAIN.setDefaultCommand(new DriveCommand(DRIVETRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        JoystickButton ResetHeading = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.RESET_HEADING);
        ResetHeading.whenPressed(new InstantCommand(DRIVETRAIN::resetHeading, DRIVETRAIN));
/*
        JoystickButton ApriltagAllignLeft = new JoystickButton(JOYSTICK_DRIVE_LEFT, Constants.OI.ALLIGN_LEFT);
        ApriltagAllignLeft.whenPressed(new allignApriltagCommand(DRIVETRAIN, VISION, allignApriltagCommand.ApriltagScoreDirection.LEFT));

        JoystickButton ApriltagAllignCenter = new JoystickButton(JOYSTICK_DRIVE_LEFT, Constants.OI.ALLIGN_LEFT);
        ApriltagAllignCenter.whenPressed(new allignApriltagCommand(DRIVETRAIN, VISION, allignApriltagCommand.ApriltagScoreDirection.CENTER));

        JoystickButton ApriltagAllignRight = new JoystickButton(JOYSTICK_DRIVE_LEFT, Constants.OI.ALLIGN_LEFT);
        ApriltagAllignRight.whenPressed(new allignApriltagCommand(DRIVETRAIN, VISION, allignApriltagCommand.ApriltagScoreDirection.RIGHT));*/
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