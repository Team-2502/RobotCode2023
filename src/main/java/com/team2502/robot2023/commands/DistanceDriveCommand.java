package com.team2502.robot2023.commands;

import com.team2502.robot2023.Utils;
import com.team2502.robot2023.Utils.Trapezoidal;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.team2502.robot2023.Constants.Subsystems.Drivetrain.*;

public class DistanceDriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;

    private Double startPos;

    private Double goalPoint;
    private Trapezoidal trapezoidal;
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
        this.trapezoidal = new Trapezoidal(LINE_T);
        pid.setTolerance(0.3);

        pid.reset();
        trapezoidal.reset();
    }

    @Override
    public void execute() {
        double error = (drivetrain.getInchesTraveled() - startPos) + goalPoint;

        double speed = pid.calculate(error);
        speed = Utils.constrain(speed, 1);
        speed = trapezoidal.calculate(speed);
        speed = Utils.frictionAdjust(speed, LINE_F);

        ChassisSpeeds speeds = new ChassisSpeeds(speed, 0, 0);
        Translation2d turn = new Translation2d(0, 0);
        drivetrain.setSpeeds(speeds, turn);
    }
}
