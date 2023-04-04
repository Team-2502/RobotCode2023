package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Arm.*;
import com.team2502.robot2023.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmSimpleCommand extends CommandBase {
    private ArmSubsystem elevator;
    private ElevatorPosition linear;
    private IntakePosition pitch;

    public SetArmSimpleCommand(ArmSubsystem elevator, ElevatorPosition pos, IntakePosition intakePosition){
        this.elevator = elevator;
        this.linear = pos;
        this.pitch = intakePosition;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.set(linear); 
        elevator.setArmPitch(pitch);
        elevator.setArmWrist(pitch);
    }

    private boolean elevatorAt(ElevatorPosition linear) {
        return Math.abs(linear.position-elevator.getLinear()) < Constants.Subsystems.Arm.ELEVATOR_THRESHOLD;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean kInterrupted) {
        elevator.setPitchSpeed(0);
        elevator.setLinearSpeed(0);
        elevator.setArmPitchSpeed(0);
        elevator.stop();
    }
}
