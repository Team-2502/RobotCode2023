package com.team2502.robot2023.commands;

import static com.team2502.robot2023.Utils.deadzone;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private PIDController rollPID;
    private boolean sideways;

    public BalanceCommand(DrivetrainSubsystem drivetrain, boolean sideways) {
        this.drivetrain = drivetrain;
        this.sideways = sideways;

        this.rollPID = new PIDController(0.02, 0, 0);
        rollPID.setTolerance(5,20);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setTurnNeutralMode(NeutralMode.Brake);
        drivetrain.setPowerNeutralMode(NeutralMode.Coast);

        ChassisSpeeds speeds;
        if (sideways) {
            speeds = new ChassisSpeeds(0, rollPID.calculate(drivetrain.getRoll()), 0);
        } else {
            speeds = new ChassisSpeeds(-rollPID.calculate(drivetrain.getPitch()), 0, 0);
        }
        drivetrain.setSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return rollPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
