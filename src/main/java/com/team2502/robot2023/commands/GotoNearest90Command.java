package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ejml.equation.IntegerSequence;

public class GotoNearest90Command extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double nearest;

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
