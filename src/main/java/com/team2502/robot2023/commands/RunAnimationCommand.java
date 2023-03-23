package com.team2502.robot2023.commands;

import com.team2502.robot2023.subsystems.LightstripSubsystem;
import com.team2502.robot2023.subsystems.LightstripSubsystem.Animation;
import com.team2502.robot2023.subsystems.LightstripSubsystem.ScheduledAnimation;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunAnimationCommand extends CommandBase {
    LightstripSubsystem lightstrip;
    Animation animation;
    ScheduledAnimation handle;
    int order;

    public RunAnimationCommand(LightstripSubsystem lightstrip, Animation animation, int order) {
        this.lightstrip = lightstrip;
        this.animation = animation;
        this.order = order;
    }

    @Override
    public void initialize() {
        handle = lightstrip.scheduleAnimation(animation, order);
    }

    @Override
    public void end(boolean interrupted) {
        lightstrip.cancelAnimation(handle);;
    }
}