package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Utils;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPosition;
import com.team2502.robot2023.Constants.Subsystems.Arm.IntakePosition;
import com.team2502.robot2023.subsystems.ArmSubsystem;
import com.team2502.robot2023.subsystems.DrivetrainSubsystem;
import com.team2502.robot2023.subsystems.PhotonVisionSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoPickupCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private PhotonVisionSubsystem vision;
    private boolean cube;

    private PIDController xPidController;
    private PIDController yPidController;
    private PIDController rPidController;

    private enum State {
        READY_INTAKE,
        ADVANCE,
        PICKUP
    }
    private State state;

    SetArmSimpleCommand setArm;

    public AutoPickupCommand(DrivetrainSubsystem drivetrain, ArmSubsystem arm, boolean cube) {
        this.drivetrain = drivetrain;
        this.vision = drivetrain.vision;

        // initialize PIDs
        this.xPidController = new PIDController(Drivetrain.DRIVETRAIN_MOVE_P, Drivetrain.DRIVETRAIN_MOVE_I, Drivetrain.DRIVETRAIN_MOVE_D);
        this.yPidController = new PIDController(Drivetrain.DRIVETRAIN_MOVE_P, Drivetrain.DRIVETRAIN_MOVE_I, Drivetrain.DRIVETRAIN_MOVE_D);
        this.rPidController = new PIDController(Drivetrain.DRIVETRAIN_TURN_P, Drivetrain.DRIVETRAIN_TURN_I, Drivetrain.DRIVETRAIN_TURN_D);

        setArm = new SetArmSimpleCommand(arm, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xPidController.setSetpoint(0);
        yPidController.setSetpoint(0);
        rPidController.setSetpoint(0);

        state = State.READY_INTAKE;
        setArm.schedule();
    }

    @Override
    public void execute() {

        switch (state) {
        case READY_INTAKE:
            if (setArm.isFinished()) {state = State.ADVANCE;}
            break;
        case ADVANCE:
            break;
        case PICKUP:
            break;
        }

        double xPower = 0;
        double yPower = 0;
        double rPower = 0;

        if (vision.gamePiece != null && vision.gamePiece.isPresent() && vision.gamePiece.get() != null) { 

            xPower = (vision.gamePiece.get().getYaw()-8)/3;
            //yPower = -yPidController.calculate(dist);
            yPower = 0.9;
            rPower = -(vision.gamePiece.get().getYaw()-8)*2;

            SmartDashboard.putNumber("GTA xp", xPower);
            SmartDashboard.putNumber("GTA yp", yPower);
        } 

        ChassisSpeeds speeds = new ChassisSpeeds(-3, xPower, rPower);
        drivetrain.setSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        setArm.cancel();
    }
}
