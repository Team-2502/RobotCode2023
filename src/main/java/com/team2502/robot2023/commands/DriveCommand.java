package com.team2502.robot2023.commands;

import static com.team2502.robot2023.Utils.deadzone;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    private final XboxController controller;

    private enum DriveController {
        Joystick,
        Xbox,
    }

    private enum Drivetype {
        FieldOriented,
        RobotOriented,
	    FieldOrientedTwist,
        RobotOrientedCenteredRot,
        VirtualTank,
        VirtualSplitArcade,
    }

    private final SendableChooser<Drivetype> typeEntry = new SendableChooser<>();
    private final SendableChooser<DriveController> controllerEntry = new SendableChooser<>();

    private final byte[] m_axes = new byte[Joystick.AxisType.values().length];

    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick joystickDriveLeft, Joystick joystickDriveRight, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        leftJoystick = joystickDriveLeft;
        rightJoystick = joystickDriveRight;

        typeEntry.addOption("Split Arcade", Drivetype.VirtualSplitArcade);
        typeEntry.addOption("Adhi mode", Drivetype.VirtualTank);
        typeEntry.addOption("Field Oriented", Drivetype.FieldOriented);
        typeEntry.addOption("Nolan mode", Drivetype.RobotOrientedCenteredRot);
	    typeEntry.addOption("Field Twist", Drivetype.FieldOrientedTwist);
        typeEntry.setDefaultOption("Robot Oriented", Drivetype.RobotOriented);
        SmartDashboard.putData("Drive Type", typeEntry);

        controllerEntry.addOption("Joystick", DriveController.Joystick);
        controllerEntry.addOption("Xbox", DriveController.Xbox);
        SmartDashboard.putData("Drive Controller", controllerEntry);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds;
        Translation2d centerOfRotation;
        drivetrain.setTurnNeutralMode(NeutralMode.Brake);
        drivetrain.setPowerNeutralMode(NeutralMode.Coast);

        if (controllerEntry.getSelected() == DriveController.Xbox) {
            switch(typeEntry.getSelected()) {
                case RobotOriented:
                    speeds = new ChassisSpeeds(deadzone(-controller.getLeftY()) * Drivetrain.MAX_VEL, deadzone(-controller.getLeftX())* Drivetrain.MAX_VEL, deadzone(controller.getRightX())*Drivetrain.MAX_ROT);
                    //centerOfRotation = new Translation2d(deadzone(controller.getRightY()),deadzone(controller.getRightX()));
                    centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                    break;
                case FieldOriented:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            deadzone(-controller.getLeftY()) * Drivetrain.MAX_VEL,
                            deadzone(-controller.getLeftX()) * Drivetrain.MAX_VEL,
                            deadzone(controller.getRightX()) * Drivetrain.MAX_ROT,
                            Rotation2d.fromDegrees(deadzone(-drivetrain.getHeading())));
                    centerOfRotation = new Translation2d(deadzone(controller.getRightY()),deadzone(controller.getRightX()));
                    //centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
            }
        } else {
            switch(typeEntry.getSelected()) {
                case RobotOriented:
                    speeds = new ChassisSpeeds(leftJoystick.getY()* Drivetrain.MAX_VEL, -leftJoystick.getX()* Drivetrain.MAX_VEL, -rightJoystick.getZ()*Drivetrain.MAX_ROT);
                    centerOfRotation = new Translation2d(rightJoystick.getY(),rightJoystick.getX());
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                    break;
                case RobotOrientedCenteredRot:
                    speeds = new ChassisSpeeds(-leftJoystick.getY()* Drivetrain.MAX_VEL, -leftJoystick.getX()* Drivetrain.MAX_VEL, rightJoystick.getX()*Drivetrain.MAX_ROT);
                    centerOfRotation = new Translation2d(0,0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                    break;
                case FieldOriented:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-leftJoystick.getY() * Drivetrain.MAX_VEL, -leftJoystick.getX() * Drivetrain.MAX_VEL, rightJoystick.getX() * Drivetrain.MAX_ROT, Rotation2d.fromDegrees(-drivetrain.getHeading()));
                    centerOfRotation = new Translation2d(rightJoystick.getY(),rightJoystick.getX());
                    //centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                case FieldOrientedTwist:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            leftJoystick.getY() * Drivetrain.MAX_VEL,
                            -leftJoystick.getX() * Drivetrain.MAX_VEL,
                            -rightJoystick.getZ() * Drivetrain.MAX_ROT,
                            Rotation2d.fromDegrees(drivetrain.getHeading()));
                    centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
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
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
