package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Intake.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax pitchIntake;
    private CANSparkMax intake;

    private SparkMaxPIDController pid;

    public IntakeSubsystem() {
        pitchIntake = new CANSparkMax(Constants.HardwareMap.INTAKE_PITCH, CANSparkMaxLowLevel.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.HardwareMap.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);

        pitchIntake.setSmartCurrentLimit(39);
        intake.setSmartCurrentLimit(39);

        pid = pitchIntake.getPIDController();
        setupPid();
    }

    private void setupPid() {
        pid.setP(Constants.Subsystems.Intake.INTAKE_P);
        pid.setI(Constants.Subsystems.Intake.INTAKE_I);
        pid.setD(Constants.Subsystems.Intake.INTAKE_D);
        pitchIntake.burnFlash();
    }

    public void setPitch(IntakePosition position) {
        pitchIntake.getPIDController().setReference(position.position, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        intake.set(speed);
    }

    public void zero() {
        pitchIntake.getEncoder().setPosition(0);
    }
}
