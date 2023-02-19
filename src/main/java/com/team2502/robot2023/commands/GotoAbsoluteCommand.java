
package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GotoAbsoluteCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private PIDController xPidController;
    private PIDController yPidController;
    private PIDController rPidController;

    public GotoAbsoluteCommand(DrivetrainSubsystem drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;

        this.xPidController = new PIDController(0.25, 0, 0);
        this.yPidController = new PIDController(0.25, 0, 0);
        this.rPidController = new PIDController(0.5, 0, 0);

        this.xPidController.setSetpoint(goalPose.getX());
        this.yPidController.setSetpoint(goalPose.getY());
        this.rPidController.setSetpoint(goalPose.getRotation().getRadians());

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d odometry = drivetrain.getPose();
        double xPower = -xPidController.calculate(odometry.getX());
        double yPower = -yPidController.calculate(odometry.getY());
        double rPower = -rPidController.calculate(odometry.getRotation().getRadians());

        ChassisSpeeds speeds = new ChassisSpeeds(xPower, yPower, rPower);
        Translation2d centerOfRotation = new Translation2d(0, 0);
        drivetrain.setSpeeds(speeds, centerOfRotation);
    }

    @Override
    public boolean isFinished() {
        return false; // unimplemented
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
