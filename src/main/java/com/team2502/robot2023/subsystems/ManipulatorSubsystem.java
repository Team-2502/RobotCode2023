package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
        setupPID();

        gripper.setSoftLimit(SoftLimitDirection.kForward,(float) ManipulatorPosition.CLOSED.position);
        gripper.setSoftLimit(SoftLimitDirection.kReverse,(float) ManipulatorPosition.OPEN.position);
        gripper.enableSoftLimit(SoftLimitDirection.kForward,true);
        gripper.enableSoftLimit(SoftLimitDirection.kReverse,true);

        limitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_GRIPPER);

        SmartDashboard.putData("Test: Gripper CLOSE", new InstantCommand(() -> { set(ManipulatorPosition.CLOSED);}));
        SmartDashboard.putData("Test: Gripper CONE", new InstantCommand(() -> { set(ManipulatorPosition.CONE);}));
        SmartDashboard.putData("Test: Gripper CUBE", new InstantCommand(() -> { set(ManipulatorPosition.CUBE);}));
        SmartDashboard.putData("Test: Gripper OPEN", new InstantCommand(() -> { set(ManipulatorPosition.OPEN);}));
        SmartDashboard.putData("Test: Gripper HOME", new InstantCommand(() -> { home();}));
    }

    private void setupPID() {
        SparkMaxPIDController pid = gripper.getPIDController();

        pid.setP(Constants.Subsystems.Manipulator.GRIPPER_P);
        pid.setI(Constants.Subsystems.Manipulator.GRIPPER_I);
        pid.setD(Constants.Subsystems.Manipulator.GRIPPER_D);
        gripper.burnFlash();
    }

    public void set(ManipulatorPosition pos) {
        gripper.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        gripper.set(speed);
    }

    public void home() {
        gripper.enableSoftLimit(SoftLimitDirection.kReverse,false);
        while (limitSwitch.get()) {
            setSpeed(-0.1);
        }

        gripper.getEncoder().setPosition(0);
        set(ManipulatorPosition.OPEN);
        gripper.enableSoftLimit(SoftLimitDirection.kReverse,true);
    }

    public void stop() {
        gripper.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("grip pos", gripper.getEncoder().getPosition());
        SmartDashboard.putBoolean("grip lim", limitSwitch.get());
    }
}
