package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Intake.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    //Define motors
    private final CANSparkMax leftLiftMotor;
    private final CANSparkMax rightLiftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private DigitalInput leftLimitSwitch;
    private DigitalInput rightLimitSwitch;


    public IntakeSubsystem() {
        // Pulls Motor Info From Constants and Defines Motor Type
        leftLiftMotor = new CANSparkMax(Constants.HardwareMap.LEFT_LIFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightLiftMotor = new CANSparkMax(Constants.HardwareMap.RIGHT_LIFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.HardwareMap.RIGHT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.HardwareMap.LEFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Sets Speed Limit To Motors
        leftLiftMotor.setSmartCurrentLimit(39);
        rightLiftMotor.setSmartCurrentLimit(39);
        rightMotor.setSmartCurrentLimit(39);
        leftMotor.setSmartCurrentLimit(39);

        //Sets Left Motor To Spin Opposite Direction To Work With Right Motor
        leftMotor.setInverted(true);

        leftLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftLimitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_LEFT_INTAKE);
        rightLimitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_RIGHT_INTAKE);
    }

    public void set(IntakePosition pos) {
        leftLiftMotor.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kSmartMotion);
        rightLiftMotor.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void run(double leftSpeed, double rightSpeed) {
        leftMotor.set(-leftSpeed);
        rightMotor.set(-rightSpeed);
    }

    void runLeft(double speed) {
        leftMotor.set(-speed);
    }

    void runRight(double speed) {
        rightMotor.set(-speed);
    }

    public void home() {
        while (leftLimitSwitch.get()) {
            runLeft(-0.1);
        }

        while (rightLimitSwitch.get()) {
            runRight(-0.1);
        }

        set(IntakePosition.RETRACTED);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void stopLift() {
        leftLiftMotor.stopMotor();
        rightLiftMotor.stopMotor();
    }
}
