package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.ElevatorSubsystem;
import com.team2502.robot2023.subsystems.IntakeSubsystem;
import com.team2502.robot2023.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HomeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private ManipulatorSubsystem manipulator;

    public HomeCommand(IntakeSubsystem intake, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator) {
        this.intake = intake;
        this.elevator = elevator;
        this.manipulator = manipulator;

        addRequirements(intake, elevator, manipulator);
    }

    @Override
    public void execute() {
        // TODO get limit switches on robot and add to subsystems

        /*
        Order:
        1. Ensure that manipulator is in position where elevator can extend/retract
        2. Left/Right intake
        3. Elevator, then extend to max
        4. Manipulator, then return elevator
         */
    }
}
