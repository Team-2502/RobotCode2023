package com.team2502.robot2023.commands;

import java.io.IOException;

import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FollowPathAbsoluteCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private Trajectory path;

    private Timer elapsed;

    public FollowPathAbsoluteCommand(DrivetrainSubsystem drivetrain, String name) {
        this.drivetrain = drivetrain;
        this.elapsed = new Timer();

        try {
			this.path = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("output/"+name+".wpilib.json"));
		} catch (IOException e) {
            // unrecoverable, will throw npe eventually
			e.printStackTrace();
		}

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        elapsed.reset();
        elapsed.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("FPR ix", path.sample(0).poseMeters.getX());
        SmartDashboard.putNumber("FPR iy", path.sample(0).poseMeters.getY());
        SmartDashboard.putNumber("FPR cx", path.sample(elapsed.get()).poseMeters.getX());
        SmartDashboard.putNumber("FPR cy", path.sample(elapsed.get()).poseMeters.getY());
        SmartDashboard.putNumber("FPR el", elapsed.get());

        
        if (elapsed.get() < path.getTotalTimeSeconds()) {
            Pose2d targ = path.sample(elapsed.get()).poseMeters;
            drivetrain.setGoalPose(new Pose2d(targ.getX(),targ.getY(),Rotation2d.fromDegrees(180)));
        }
    }

    @Override
    public boolean isFinished() {
        return (elapsed.get() > path.getTotalTimeSeconds()) && drivetrain.atGoalPose();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
