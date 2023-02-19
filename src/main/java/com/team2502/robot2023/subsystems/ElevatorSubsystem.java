package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team2502.robot2023.Constants;
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

        double kP = 5e-5;
        double kI = 1e-6;
        double kD = 0;
        double kIz = 0;
        double kFF = 0.000156;
        double kMaxOutput = 1;
        double kMinOutput = -1;

        double maxVel = 2000;
        double minVel = 0;
        double maxAcc = 1500;
        double allowedErr = 5;

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        pid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pid.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }

    public void set(ElevatorPosition pos) {
        switch (pos) {
            case BOTTOM:
                rightElevator.getPIDController().setReference(0, CANSparkMax.ControlType.kSmartMotion);
            case MIDDLE:
                rightElevator.getPIDController().setReference(0, CANSparkMax.ControlType.kSmartMotion);
            case TOP:
                rightElevator.getPIDController().setReference(0, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    public void stop() {
        rightElevator.stopMotor();
        leftElevator.stopMotor();
    }
}
