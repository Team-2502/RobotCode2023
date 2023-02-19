package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.ElevatorSubsystem;
import com.team2502.robot2023.subsystems.ElevatorSubsystem.ElevatorPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunElevatorCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private ElevatorPosition pos;

    public RunElevatorCommand(ElevatorSubsystem elevator, ElevatorPosition pos) {
        this.elevator = elevator;
        this.pos = pos;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.set(pos);
    }

    @Override
    public void end(boolean kInterrupted) {
        elevator.stop();
    }
}
