package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Intake.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax pitchIntake;
    private CANSparkMax intake;

    private SparkMaxPIDController pid;

    private DigitalInput limitSwitch;

    public IntakeSubsystem() {
        pitchIntake = new CANSparkMax(Constants.HardwareMap.INTAKE_PITCH, CANSparkMaxLowLevel.MotorType.kBrushless);
        intake = new CANSparkMax(Constants.HardwareMap.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);

        //limitSwitch = new DigitalInput(Constants.HardwareMap.INTAKE_PITCH);

        pitchIntake.setSmartCurrentLimit(39);
        intake.setSmartCurrentLimit(39);

        pitchIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pid = pitchIntake.getPIDController();
        setupPid();
    }

    @Override
    public void periodic() {
        double wristAngle = (pitchIntake.getEncoder().getPosition() * Constants.Subsystems.Intake.WRIST_ROT_TO_DEGREE) + Constants.Subsystems.Intake.WRIST_ZERO_ANGLE;
        SmartDashboard.putNumber("elbow ang raw", pitchIntake.getEncoder().getPosition());
        SmartDashboard.putNumber("elbow ang", wristAngle);
    }

    private void setupPid() {
        pid.setP(Constants.Subsystems.Intake.INTAKE_P);
        pid.setI(Constants.Subsystems.Intake.INTAKE_I);
        pid.setD(Constants.Subsystems.Intake.INTAKE_D);
        pitchIntake.burnFlash();
    }

    public void setPitch(IntakePosition position) {
        pitchIntake.getPIDController().setReference(position.pitchElbow, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        intake.set(speed);
    }

    public void setPitchSpeed(double speed) {
        pitchIntake.set(speed);
    }

    public void home() {
        while (!limitSwitch.get()) {
            setPitchSpeed(0.1);
            if (pitchIntake.getOutputCurrent() > 15) {
                break;
            }
        }

        pitchIntake.getEncoder().setPosition(0);
        setPitch(IntakePosition.IN);
    }

    public void zero() {
        pitchIntake.getEncoder().setPosition(0);
    }
}
