package com.team2502.robot2023.commands;

import com.team2502.robot2023.Utils.Range;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GotoNearest90Command extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double nearest;

    private Range straight;
    private Range left;
    private Range right;
    private Range back;

    public GotoNearest90Command(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }
}
