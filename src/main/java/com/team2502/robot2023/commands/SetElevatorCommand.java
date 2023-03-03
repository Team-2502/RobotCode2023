package com.team2502.robot2023.commands;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Elevator.*;
import com.team2502.robot2023.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private ElevatorPosition linear;
    private ElevatorPitch pitch;
    private boolean reposition;

    public SetElevatorCommand(ElevatorSubsystem elevator, ElevatorPosition pos) {
        this.elevator = elevator;
        this.linear = pos;
        this.pitch = ElevatorPitch.STOWED;
        this.reposition = true;

        addRequirements(elevator);
    }

    public SetElevatorCommand(ElevatorSubsystem elevator, ElevatorPosition pos, ElevatorPitch pitch) {
        this.elevator = elevator;
        this.linear = pos;
        this.pitch = pitch;
        this.reposition = true;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // calculate if this manuver will cross the deadzone caused by the back of the frame
        // lang3.Range is not installed
        reposition = (elevator.getPitch() > ElevatorPitch.FRAME_INTERSECT.position && pitch.position < ElevatorPitch.FRAME_INTERSECT.position)
            || (elevator.getPitch() < ElevatorPitch.FRAME_INTERSECT.position && pitch.position > ElevatorPitch.FRAME_INTERSECT.position);
    }

    @Override
    public void execute() {
        if (!reposition) {
            elevator.setPitch(pitch);
            elevator.set(linear);
            return;
        }

        // repositioning
        elevator.set(ElevatorPosition.SAFE_PITCH); // extend (presumably) to the safepoint
        if (elevatorAt(ElevatorPosition.SAFE_PITCH)) {
            elevator.setPitch(pitch); // pitch over once safe
            if (pitchAt(pitch)) {
                reposition = false; // end repositioning mode once at setpoint
            }
        }
    }

    private boolean elevatorAt(ElevatorPosition linear) {
        return Math.abs(linear.position-elevator.getLinear()) < Constants.Subsystems.Elevator.ELEVATOR_THRESHOLD;
    }

    private boolean pitchAt(ElevatorPitch pitch) {
        return Math.abs(pitch.position-elevator.getPitch()) < Constants.Subsystems.Elevator.PITCH_THRESHOLD;
    }

    @Override
    public boolean isFinished() {
        if (elevatorAt(linear) && pitchAt(pitch)) {
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
