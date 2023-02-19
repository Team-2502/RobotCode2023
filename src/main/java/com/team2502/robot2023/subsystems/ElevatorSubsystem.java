package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team2502.robot2023.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax leftElevator;
    private CANSparkMax rightElevator;
    private SparkMaxPIDController pid;
    private RelativeEncoder encoder;

    public enum ElevatorPosition {
        BOTTOM,
        MIDDLE,
        TOP
    }

    public ElevatorSubsystem() {
        leftElevator = new CANSparkMax(Constants.HardwareMap.LEFT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightElevator = new CANSparkMax(Constants.HardwareMap.RIGHT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftElevator.follow(rightElevator, true);

        leftElevator.setSmartCurrentLimit(39);
        rightElevator.setSmartCurrentLimit(39);

        pid = rightElevator.getPIDController();
        encoder = rightElevator.getEncoder();
    }

    public void set(ElevatorPosition pos) {
        switch (pos) {
            case BOTTOM:
                pid.setReference(0, CANSparkMax.ControlType.kSmartMotion);
            case MIDDLE:
                pid.setReference(0, CANSparkMax.ControlType.kSmartMotion);
            case TOP:
                pid.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    public void stop() {
        rightElevator.stopMotor();
        leftElevator.stopMotor();
    }
}
