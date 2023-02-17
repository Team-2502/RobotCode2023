package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import static com.team2502.robot2023.Constants.Subsystems.Drivetrain.*;

public class DistanceDriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;

    private Double startPos;
    private SlewRateLimiter
    private Double goalPoint;
    private PIDController pid;

    public DistanceDriveCommand(DrivetrainSubsystem drivetrain, Double goalPoint) {
        this.drivetrain = drivetrain;
        this.goalPoint = goalPoint;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.startPos = drivetrain.getInchesTraveled();
        this.pid = new PIDController(LINE_P, LINE_I, LINE_D);
        pid.setTolerance(0.3);

        pid.reset();
    }

    @Override
    public void execute() {
        double error = (drivetrain.getInchesTraveled() - startPos) + goalPoint;

        double speed = pid.calculate(error);
    }
}
