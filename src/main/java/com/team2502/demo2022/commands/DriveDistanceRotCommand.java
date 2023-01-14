package com.team2502.demo2022.commands;

import com.team2502.demo2022.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2502.demo2022.Constants.Subsystems.Drivetrain;

public class DriveDistanceRotCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double movePoint;
    private Rotation2d rotPoint;

    private double startPos;
    private PIDController pid;
    private PIDController turnPID;

    public DriveDistanceRotCommand(DrivetrainSubsystem drivetrain, double movePoint, Rotation2d rotPoint) {
        this.drivetrain = drivetrain;
        this.movePoint = movePoint;
        this.rotPoint = rotPoint;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setToDistance(0.0001);
    }

    //@Override
    //public void end(boolean interrupted) {
    //    drivetrain.stop();
    //}

    @Override
    public boolean isFinished() {
        return false;
    }
}
