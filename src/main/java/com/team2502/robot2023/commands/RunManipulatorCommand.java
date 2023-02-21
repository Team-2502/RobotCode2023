package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunManipulatorCommand extends CommandBase {
    private ManipulatorSubsystem manipulator;
    private double val;

    public RunManipulatorCommand(ManipulatorSubsystem manipulator, double val) {
        this.manipulator = manipulator;

        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        manipulator.setSpeed(val);
    }
}
