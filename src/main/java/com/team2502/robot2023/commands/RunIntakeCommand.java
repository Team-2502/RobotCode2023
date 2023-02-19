package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.ConveyorSubsystem;
import com.team2502.robot2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private ConveyorSubsystem conveyor;
    private double intakeLeftSpeed;
    private double intakeRightSpeed;
    private double conveyorSpeed;

    public RunIntakeCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor, double intakeLeftSpeed, double intakeRightSpeed, double conveyorSpeed) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.intakeLeftSpeed = intakeLeftSpeed;
        this.intakeRightSpeed = intakeRightSpeed;
        this.conveyorSpeed = conveyorSpeed;

        addRequirements(intake, conveyor);
    }

    @Override
    public void execute() {
        intake.run(intakeLeftSpeed, intakeRightSpeed);
        conveyor.run(conveyorSpeed);
    }

    @Override
    public void end(boolean kInterrupted) {
        conveyor.stop();
        intake.stop();
    }
}
