package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GotoNearestScoreCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private Pose2d nearest;

    public GotoNearestScoreCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // TODO find nearest position
    }

    @Override
    public void execute() {
        drivetrain.setGoalPose(nearest);
    }
}
