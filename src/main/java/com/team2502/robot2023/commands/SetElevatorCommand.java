package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Elevator.*;
import com.team2502.robot2023.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private ElevatorPosition pos;

    public SetElevatorCommand(ElevatorSubsystem elevator, ElevatorPosition pos) {
        this.elevator = elevator;
        this.pos = pos;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (!elevator.safePitch() && !pitchStowed()) {
            elevator.setLinearSpeed(0);
        } else {
            elevator.set(pos);
        }

        elevator.setPitch(elevator.safePitch() ? ElevatorPitch.OUT : ElevatorPitch.STOWED);
    }

    private boolean elevatorAtSetpoint() {
        return Math.abs(pos.position-elevator.getLinear()) < Constants.Subsystems.Elevator.ELEVATOR_THRESHOLD;
    }

    private boolean pitchStowed() {
        return Math.abs(ElevatorPitch.STOWED.position-elevator.getPitch()) < Constants.Subsystems.Elevator.PITCH_THRESHOLD;
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(pos.position-elevator.getLinear()) < Constants.Subsystems.Elevator.ELEVATOR_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean kInterrupted) {
        elevator.setPitchSpeed(0);
        elevator.setLinearSpeed(0);
        elevator.stop();
    }
}
