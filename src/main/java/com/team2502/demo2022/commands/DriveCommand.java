package com.team2502.demo2022.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.demo2022.Constants.Subsystems.Drivetrain;
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
        RobotOrientedCenteredRot,
        VirtualTank,
        VirtualSplitArcade
    }

    private final SendableChooser<Drivetype> typeEntry = new SendableChooser<>();

    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick joystickDriveLeft, Joystick joystickDriveRight) {
        this.drivetrain = drivetrain;
        leftJoystick = joystickDriveLeft;
        rightJoystick = joystickDriveRight;

        typeEntry.addOption("Split Arcade", Drivetype.VirtualSplitArcade);
        typeEntry.addOption("adhi mode", Drivetype.VirtualTank);
        typeEntry.addOption("Field Oriented", Drivetype.FieldOriented);
        typeEntry.addOption("nolan mode", Drivetype.RobotOrientedCenteredRot);
        typeEntry.setDefaultOption("Robot Oriented", Drivetype.RobotOriented);
        SmartDashboard.putData("Drive Type", typeEntry);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds;
        Translation2d centerOfRotation;
        drivetrain.setTurnNeutralMode(NeutralMode.Brake);
        drivetrain.setPowerNeutralMode(NeutralMode.Coast);
        switch(typeEntry.getSelected()) {
            case RobotOriented:
                speeds = new ChassisSpeeds(-leftJoystick.getY()* Drivetrain.MAX_VEL, -leftJoystick.getX()* Drivetrain.MAX_VEL, rightJoystick.getZ()*Drivetrain.MAX_ROT);
                centerOfRotation = new Translation2d(rightJoystick.getY(),rightJoystick.getX());
                drivetrain.setSpeeds(speeds, centerOfRotation);
                break;
            case RobotOrientedCenteredRot:
                speeds = new ChassisSpeeds(-leftJoystick.getY()* Drivetrain.MAX_VEL, -leftJoystick.getX()* Drivetrain.MAX_VEL, rightJoystick.getX()*Drivetrain.MAX_ROT);
                centerOfRotation = new Translation2d(0,0);
                drivetrain.setSpeeds(speeds, centerOfRotation);
                break;
            case FieldOriented:
                // TODO: impl after initial test
                break;
            case VirtualTank:
                speeds = new ChassisSpeeds(-(leftJoystick.getY()+rightJoystick.getY())* Drivetrain.MAX_VEL, 0, (rightJoystick.getY()-leftJoystick.getY())*Drivetrain.MAX_ROT);
                centerOfRotation = new Translation2d(0,0);
                drivetrain.setSpeeds(speeds, centerOfRotation);
                break;
            case VirtualSplitArcade:
                speeds = new ChassisSpeeds(-leftJoystick.getY()* Drivetrain.MAX_VEL, 0, rightJoystick.getX()*Drivetrain.MAX_ROT);
                centerOfRotation = new Translation2d(0,0);
                drivetrain.setSpeeds(speeds, centerOfRotation);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
