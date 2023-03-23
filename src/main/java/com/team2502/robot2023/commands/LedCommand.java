package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Leds.*;
import com.team2502.robot2023.subsystems.ArmSubsystem;
import com.team2502.robot2023.subsystems.IntakeSubsystem;
import com.team2502.robot2023.subsystems.LightstripSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj.RobotState.isDisabled;
import static edu.wpi.first.wpilibj.RobotState.isEnabled;

public class LedCommand extends CommandBase {
    private LightstripSubsystem lightstrip;
    private IntakeSubsystem intake;
    private ArmSubsystem arm;

    public LedCommand(LightstripSubsystem lightstrip, IntakeSubsystem intake, ArmSubsystem arm) {
        this.lightstrip = lightstrip;
        this.intake = intake;
        this.arm = arm;

        addRequirements(lightstrip);
    }

    @Override
    public void execute() {
        if (isDisabled()) {
            lightstrip.scheduleAnimation(LightstripSubsystem.Animations.disabled, 1);
        } if (isEnabled() && intake.isRunning()) {
            lightstrip.scheduleAnimation(LightstripSubsystem.Animations.intake, 1);
        }
    }
}
