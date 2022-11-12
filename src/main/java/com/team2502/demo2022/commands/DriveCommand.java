package com.team2502.demo2022.commands;

import com.team2502.demo2022.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    private enum Drivetype {
        FieldOriented,
        RobotOriented,
        VirtualTank,
    }

    private final SendableChooser<Drivetype> typeEntry = new SendableChooser<>();

    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick joystickDriveLeft, Joystick joystickDriveRight) {
        this.drivetrain = drivetrain;
        leftJoystick = joystickDriveLeft;
        rightJoystick = joystickDriveRight;

        typeEntry.addOption("Tank", Drivetype.VirtualTank);
        typeEntry.addOption("Field Oriented", Drivetype.FieldOriented);
        typeEntry.setDefaultOption("Robot Oriented", Drivetype.RobotOriented);
        SmartDashboard.putData("Drive Type", typeEntry);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        switch(typeEntry.getSelected()) {
            case RobotOriented:
                ChassisSpeeds speeds = new ChassisSpeeds(leftJoystick.getX(), leftJoystick.getY(), rightJoystick.getZ());
                Translation2d centerOfRotation = new Translation2d(rightJoystick.getX(),rightJoystick.getY());
                drivetrain.setSpeeds(speeds, centerOfRotation);
                break;
            case FieldOriented:
                // TODO: impl after initial test
                break;
            case VirtualTank:
                // TODO: impl after initial test
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
