package com.team2502.robot2023.commands;

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
        elevator.set(pos);

        if (elevator.safePitch()) {
            elevator.setPitch(ElevatorPitch.OUT);
        }
    }

    @Override
    public boolean isFinished() {
        if (elevator.getLinear() <= elevator.getLinear() + 1 && elevator.getLinear() >= elevator.getLinear() - 1) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean kInterrupted) {
        elevator.stop();
    }
}
