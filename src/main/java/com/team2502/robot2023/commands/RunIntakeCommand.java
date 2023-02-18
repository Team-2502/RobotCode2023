package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double speed;

    public RunIntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.run(speed);
    }

    @Override
    public void end(boolean kInterrupted) {
        intake.stop();
    }
}
