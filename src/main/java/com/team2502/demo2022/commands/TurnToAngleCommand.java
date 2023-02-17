package com.team2502.demo2022.commands;

import com.team2502.demo2022.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngleCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double goalPoint;

    public TurnToAngleCommand(DrivetrainSubsystem drivetrain, double goalPoint) {
        this.drivetrain = drivetrain;
        this.goalPoint = goalPoint;

        addRequirements(drivetrain);
    }
}
