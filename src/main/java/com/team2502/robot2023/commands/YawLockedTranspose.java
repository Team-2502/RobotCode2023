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

    public YawLockedTranspose(DrivetrainSubsystem drivetrain, ChassisSpeeds speeds) {
        this.drivetrain = drivetrain;
        yawGoal = drivetrain.getHeading();
        this.speeds = speeds;
        this.rPidController = new PIDController(Drivetrain.DRIVETRAIN_TURN_P, Drivetrain.DRIVETRAIN_TURN_I, Drivetrain.DRIVETRAIN_TURN_D);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        yawGoal = drivetrain.getHeading();
    }

    @Override
    public void execute() {
        double rPower = -rPidController.calculate(Units.degreesToRadians(drivetrain.getHeading()-yawGoal));
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
