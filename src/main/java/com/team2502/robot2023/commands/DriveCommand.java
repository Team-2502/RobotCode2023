package com.team2502.robot2023.commands;

import static com.team2502.robot2023.Utils.deadzone;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.robot2023.Constants.OI;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import com.team2502.robot2023.Utils;

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
    //private final XboxController controller;

    private enum DriveController {
        Joystick,
        Xbox,
    }

    private enum Drivetype {
        FieldOriented,
        RobotOriented,
	    FieldOrientedTwist,
	    FieldOrientedTwistRet,
	    FieldOrientedTwistRetDead,
	    FieldOrientedTwistPow,
        RobotOrientedCenteredRot,
        VirtualTank,
        VirtualSplitArcade,
    }

    private double dlxDrift;
    private double dlyDrift;

    private boolean slowmode =  false;
    private boolean prevTog = false;

    private final SendableChooser<Drivetype> typeEntry = new SendableChooser<>();
    private final SendableChooser<DriveController> controllerEntry = new SendableChooser<>();

    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick joystickDriveLeft, Joystick joystickDriveRight) {
        this.drivetrain = drivetrain;
        //this.controller = controller;
        leftJoystick = joystickDriveLeft;
        rightJoystick = joystickDriveRight;

        typeEntry.addOption("Split Arcade", Drivetype.VirtualSplitArcade);
        typeEntry.addOption("adhi mode", Drivetype.VirtualTank);
        typeEntry.addOption("Field Oriented", Drivetype.FieldOriented);
        typeEntry.addOption("nolan mode", Drivetype.RobotOrientedCenteredRot);
        typeEntry.addOption("Robot Oriented", Drivetype.RobotOriented);
        typeEntry.addOption("Field Pow", Drivetype.FieldOrientedTwistPow);
	    typeEntry.addOption("Field Twist", Drivetype.FieldOrientedTwist);
	    typeEntry.addOption("Super Loud Olympic Winning", Drivetype.FieldOrientedTwistRet);
	    typeEntry.setDefaultOption("Dead", Drivetype.FieldOrientedTwistRetDead);
        SmartDashboard.putData("Drive Type", typeEntry);

        controllerEntry.addOption("Joystick", DriveController.Joystick);
        controllerEntry.addOption("Xbox", DriveController.Xbox);
        SmartDashboard.putData("Drive Controller", controllerEntry);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (leftJoystick.getRawButton(OI.DRIFT_RESET)) {
            resetDrift();
        }

        boolean tog = leftJoystick.getRawButton(OI.RET_MODE);
        if (prevTog != tog && tog) {
            slowmode = !slowmode;
        }
        prevTog = tog;

        SmartDashboard.putBoolean("iosajioj", slowmode);

        ChassisSpeeds speeds;
        Translation2d centerOfRotation;
        drivetrain.setTurnNeutralMode(NeutralMode.Brake);
        drivetrain.setPowerNeutralMode(NeutralMode.Coast);
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
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            -(leftJoystick.getY()+dlyDrift) * Drivetrain.MAX_VEL,
                            (leftJoystick.getX()+dlxDrift) * Drivetrain.MAX_VEL,
                            rightJoystick.getX() * Drivetrain.MAX_ROT,
                            Rotation2d.fromDegrees(-drivetrain.getHeading()+drivetrain.fieldOrientedOffset));
                    centerOfRotation = new Translation2d(rightJoystick.getY(),rightJoystick.getX()).rotateBy(Rotation2d.fromDegrees(-drivetrain.getHeading()+drivetrain.fieldOrientedOffset));
                    //centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                case FieldOrientedTwistPow:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            (leftJoystick.getY() * leftJoystick.getY() * leftJoystick.getY()) * (leftJoystick.getRawButton(OI.RET_MODE) ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL),
                            -(leftJoystick.getX()*leftJoystick.getX()*leftJoystick.getX()) * (leftJoystick.getRawButton(OI.RET_MODE) ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL),
                            -(rightJoystick.getZ()*rightJoystick.getZ()*rightJoystick.getZ()) * (leftJoystick.getRawButton(OI.RET_MODE) ? Drivetrain.RET_ROT : Drivetrain.MAX_ROT),
                            Rotation2d.fromDegrees(drivetrain.getHeading()+drivetrain.fieldOrientedOffset));
                    centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                    break;
                case FieldOrientedTwist:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            leftJoystick.getY() * (slowmode ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL),
                            -leftJoystick.getX() * (slowmode ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL),
                            -rightJoystick.getZ() * (slowmode ? Drivetrain.RET_ROT : Drivetrain.MAX_ROT),
                            Rotation2d.fromDegrees(drivetrain.getHeading()+drivetrain.fieldOrientedOffset));
                    centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                    break;
                case FieldOrientedTwistRet:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            Utils.deadzone(Drivetrain.OI_DEADZONE_XY, 0.08, leftJoystick.getY()) * (slowmode ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL) * 0.65,
                            -Utils.deadzone(Drivetrain.OI_DEADZONE_XY, 0.08, leftJoystick.getX()) * (slowmode ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL) * 0.65,
                            -Utils.deadzone(Drivetrain.OI_DEADZONE_Z, 0.07, rightJoystick.getZ()) * (slowmode ? Drivetrain.RET_ROT : Drivetrain.MAX_ROT) * 0.65,
                            Rotation2d.fromDegrees(drivetrain.getHeading()+drivetrain.fieldOrientedOffset));
                    centerOfRotation = new Translation2d(0, 0);
                    drivetrain.setSpeeds(speeds, centerOfRotation);
                    break;
                case FieldOrientedTwistRetDead:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            Utils.deadzone(Drivetrain.OI_DEADZONE_XY, 0.04, leftJoystick.getY(), 0.45 * 0.6) * (slowmode ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL),
                            -Utils.deadzone(Drivetrain.OI_DEADZONE_XY, 0.04, leftJoystick.getX(), 0.45 * 0.6) * (slowmode ? Drivetrain.RET_VEL : Drivetrain.MAX_VEL),
                            -Utils.deadzone(Drivetrain.OI_DEADZONE_Z, 0.04, rightJoystick.getZ(), 0.55 * 0.6) * (slowmode ? Drivetrain.RET_ROT : Drivetrain.MAX_ROT),
                            Rotation2d.fromDegrees(drivetrain.getHeading()+drivetrain.fieldOrientedOffset));
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

    private void resetDrift() {
        dlxDrift = -leftJoystick.getX();
        dlyDrift = -leftJoystick.getY();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
