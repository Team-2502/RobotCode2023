package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax leftElevator;
    private CANSparkMax rightElevator;
    private CANSparkMax pitchElevator;

    private DigitalInput limitSwitch;

    public ElevatorSubsystem() {
        leftElevator = new CANSparkMax(Constants.HardwareMap.LEFT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightElevator = new CANSparkMax(Constants.HardwareMap.RIGHT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        pitchElevator = new CANSparkMax(Constants.HardwareMap.PITCH_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftElevator.follow(rightElevator, true);

        leftElevator.setSmartCurrentLimit(39);
        rightElevator.setSmartCurrentLimit(39);
        pitchElevator.setSmartCurrentLimit(20);

        rightElevator.setSoftLimit(SoftLimitDirection.kForward,(float) Constants.Subsystems.Elevator.ELEVATOR_LIM_BOTTOM);
        rightElevator.setSoftLimit(SoftLimitDirection.kReverse,(float) Constants.Subsystems.Elevator.ELEVATOR_LIM_TOP);
        rightElevator.enableSoftLimit(SoftLimitDirection.kForward,true);
        rightElevator.enableSoftLimit(SoftLimitDirection.kReverse,true);

        limitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_ELEVATOR);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret pos", rightElevator.getEncoder().getPosition());
    }

    public void setPitch(ElevatorPitch pitch) {
        pitchElevator.getPIDController().setReference(pitch.position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void set(ElevatorPosition pos) {
        rightElevator.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setLinearSpeed(double speed) {
        rightElevator.set(speed);
    }

    public void setPitchSpeed(double speed) {
        pitchElevator.set(speed);
    }

    public void home() {
        while (limitSwitch.get()) {
            setLinearSpeed(-0.1);
        }

        rightElevator.getEncoder().setPosition(0);
        set(ElevatorPosition.BOTTOM);
    }

    public void stop() {
        rightElevator.stopMotor();
        leftElevator.stopMotor();
        pitchElevator.stopMotor();
    }
}
