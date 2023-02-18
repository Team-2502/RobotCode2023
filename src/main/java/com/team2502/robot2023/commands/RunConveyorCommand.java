package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunConveyorCommand extends CommandBase {
    private ConveyorSubsystem conveyor;
    private double speed;

    public RunConveyorCommand(ConveyorSubsystem conveyor, double speed) {
        this.conveyor = conveyor;
        this.speed = speed;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.runConveyor(speed);
    }

    @Override
    public void end(boolean kInterrupted) {
        conveyor.stop();
    }
}
