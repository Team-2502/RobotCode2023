
package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GotoAbsoluteCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private Pose2d goalPose;

    public GotoAbsoluteCommand(DrivetrainSubsystem drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setGoalPose(goalPose);
        SmartDashboard.putBoolean("GTA ats", drivetrain.atGoalPose());
    }

    @Override
    public boolean isFinished() {
        return false;
        //return drivetrain.atGoalPose();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
