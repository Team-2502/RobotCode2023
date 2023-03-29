package com.team2502.robot2023.commands;

import java.io.IOException;

import com.team2502.robot2023.Constants.Subsystems.Drivetrain;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class YawLockedTranspose extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private double yawGoal;
    private ChassisSpeeds speeds;
    private PIDController rPidController;
    private Mode mode;
    public enum Mode {
        NAVX_ZERO,
        FIELD_ZERO,
        COMMAND_ZERO
    }

    public YawLockedTranspose(DrivetrainSubsystem drivetrain, ChassisSpeeds speeds, Mode mode) {
        this.drivetrain = drivetrain;
        yawGoal = drivetrain.getHeading();
        this.speeds = speeds;
        this.rPidController = new PIDController(Drivetrain.DRIVETRAIN_TURN_P, Drivetrain.DRIVETRAIN_TURN_I, Drivetrain.DRIVETRAIN_TURN_D);
        this.mode = mode;

        addRequirements(drivetrain);
    }

    public YawLockedTranspose(DrivetrainSubsystem drivetrain, ChassisSpeeds speeds) {
        this.drivetrain = drivetrain;
        yawGoal = drivetrain.getHeading();
        this.speeds = speeds;
        this.rPidController = new PIDController(Drivetrain.DRIVETRAIN_TURN_P, Drivetrain.DRIVETRAIN_TURN_I, Drivetrain.DRIVETRAIN_TURN_D);
        this.mode = Mode.FIELD_ZERO;

        addRequirements(drivetrain);
    }

    public YawLockedTranspose(DrivetrainSubsystem drivetrain, ChassisSpeeds speeds, boolean fieldOffset) {
        this.drivetrain = drivetrain;
        yawGoal = drivetrain.getHeading();
        this.speeds = speeds;
        this.rPidController = new PIDController(Drivetrain.DRIVETRAIN_TURN_P, Drivetrain.DRIVETRAIN_TURN_I, Drivetrain.DRIVETRAIN_TURN_D);
        this.mode = fieldOffset ? Mode.FIELD_ZERO : Mode.COMMAND_ZERO;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        switch (mode) {
            case FIELD_ZERO:
                yawGoal = drivetrain.getHeading() + drivetrain.fieldOrientedOffset;
                break;
            case NAVX_ZERO:
                yawGoal = 0;
                break;
            case COMMAND_ZERO:
                yawGoal = drivetrain.getHeading();
        }

        rPidController.setSetpoint(Units.degreesToRadians(yawGoal));
    }

    @Override
    public void execute() {
        double rPower = -rPidController.calculate(Units.degreesToRadians(drivetrain.getHeading())) * 2.0;
        drivetrain.setSpeeds(new ChassisSpeeds(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,rPower));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
