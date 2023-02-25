package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2023.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2502.robot2023.Constants.Subsystems.Manipulator.*;

public class ManipulatorSubsystem extends SubsystemBase {
    private CANSparkMax gripper;

    private DigitalInput limitSwitch;

    public ManipulatorSubsystem() {
        gripper = new CANSparkMax(Constants.HardwareMap.GRIPPER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        limitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_GRIPPER);

        SmartDashboard.putData("Test: Gripper CLOSE", new InstantCommand(() -> { set(ManipulatorPosition.CLOSED);}));
        SmartDashboard.putData("Test: Gripper OPEN", new InstantCommand(() -> { set(ManipulatorPosition.OPEN);}));
    }

    public void set(ManipulatorPosition pos) {
        gripper.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        gripper.set(speed);
    }

    public void home() {
        while (!limitSwitch.get()) {
            setSpeed(-0.1);
        }

        gripper.getEncoder().setPosition(0);
        set(ManipulatorPosition.OPEN);
    }

    public void stop() {
        gripper.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("grip pos", gripper.getEncoder().getPosition());
    }
}
