package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import com.team2502.robot2023.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class GotoNearestScoreCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private PhotonVisionSubsystem vision;

    private Pose2d nearest;
    private Pose2d current;

    public GotoNearestScoreCommand(DrivetrainSubsystem drivetrain, PhotonVisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // TODO find nearest position
        current = vision.getPose();

        //nearest = Constants.Subsystems.Field.scoreLocations.sort(((o1, o2) -> o1.getX().compareTo(o2.getX())));

        SmartDashboard.putNumber("Nearest X", nearest.getX());
    }

    @Override
    public void execute() {
        //drivetrain.setGoalPose(nearest);
    }
}
