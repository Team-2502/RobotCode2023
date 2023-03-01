package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax leftElevator;
    private CANSparkMax rightElevator;
    private CANSparkMax pitchElevator;

    private SparkMaxPIDController pid;
    private SparkMaxPIDController pitchPid;

    private DigitalInput limitSwitch;

    public ElevatorSubsystem() {
        leftElevator = new CANSparkMax(Constants.HardwareMap.LEFT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightElevator = new CANSparkMax(Constants.HardwareMap.RIGHT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        pitchElevator = new CANSparkMax(Constants.HardwareMap.PITCH_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftElevator.follow(rightElevator, true);

        leftElevator.setSmartCurrentLimit(39);
        rightElevator.setSmartCurrentLimit(39);
        pitchElevator.setSmartCurrentLimit(39);

        rightElevator.setSoftLimit(SoftLimitDirection.kForward,(float) Constants.Subsystems.Elevator.ELEVATOR_LIM_BOTTOM);
        rightElevator.setSoftLimit(SoftLimitDirection.kReverse,(float) Constants.Subsystems.Elevator.ELEVATOR_LIM_TOP);
        rightElevator.enableSoftLimit(SoftLimitDirection.kForward,true);
        rightElevator.enableSoftLimit(SoftLimitDirection.kReverse,true);

        limitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_ELEVATOR);

        pid = rightElevator.getPIDController();
        pitchPid = pitchElevator.getPIDController();
        setupPID();

        NTInit();setupPID();
        SmartDashboard.putData("Flush Elevator PIDs", new InstantCommand(() -> { NTUpdate();}));

        SmartDashboard.putData("Test: Elvator UP", new InstantCommand(() -> { set(ElevatorPosition.TOP);}));
        SmartDashboard.putData("Test: Elvator DOWN", new InstantCommand(() -> { set(ElevatorPosition.BOTTOM);}));
        SmartDashboard.putData("Test: Wrist OUT", new InstantCommand(() -> { setPitch(ElevatorPitch.OUT);}));
        SmartDashboard.putData("Test: Wrist IN", new InstantCommand(() -> { setPitch(ElevatorPitch.STOWED);}));

        SmartDashboard.putData("Elevator HOME", new InstantCommand(() -> { home();}));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elev pos", rightElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("wrist pos", pitchElevator.getEncoder().getPosition());
        if (Constants.Subsystems.Elevator.NT_TUNE) {
            NTUpdate();
        }
    }

    public void setPitch(ElevatorPitch pitch) {
        pitchElevator.getPIDController().setReference(pitch.position, CANSparkMax.ControlType.kPosition);
    }

    public void set(ElevatorPosition pos) {
        rightElevator.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kPosition);
    }

    public void setLinearSpeed(double speed) {
        rightElevator.set(speed);
    }

    public void setPitchSpeed(double speed) {
        pitchElevator.set(speed);
    }

    public void home() {
        while (!limitSwitch.get()) {
            setLinearSpeed(0.1);
            if (rightElevator.getOutputCurrent() > 15) {
                break;
            }
        }

        rightElevator.getEncoder().setPosition(0);
        set(ElevatorPosition.BOTTOM);
    }

    private void NTInit() {
        SmartDashboard.putNumber("ELEVATOR_P", Constants.Subsystems.Elevator.ELEVATOR_P);
        SmartDashboard.putNumber("ELEVATOR_I", Constants.Subsystems.Elevator.ELEVATOR_I);
        SmartDashboard.putNumber("ELEVATOR_D", Constants.Subsystems.Elevator.ELEVATOR_D);
        SmartDashboard.putNumber("PITCH_P", Constants.Subsystems.Elevator.PITCH_P);
        SmartDashboard.putNumber("PITCH_I", Constants.Subsystems.Elevator.PITCH_I);
        SmartDashboard.putNumber("PITCH_D", Constants.Subsystems.Elevator.PITCH_D);
    }

    private void setupPID() {
        pid.setP(Constants.Subsystems.Elevator.ELEVATOR_P);
        pid.setI(Constants.Subsystems.Elevator.ELEVATOR_I);
        pid.setD(Constants.Subsystems.Elevator.ELEVATOR_D);
        pid.setOutputRange(Constants.Subsystems.Elevator.ELEVATOR_MIN_OUTPUT, Constants.Subsystems.Elevator.ELEVATOR_MAX_OUTPUT);
        rightElevator.burnFlash();

        pitchPid.setP(Constants.Subsystems.Elevator.PITCH_P);
        pitchPid.setI(Constants.Subsystems.Elevator.PITCH_I);
        pitchPid.setD(Constants.Subsystems.Elevator.PITCH_D);
        pitchPid.setOutputRange(Constants.Subsystems.Elevator.PITCH_MIN_OUTPUT, Constants.Subsystems.Elevator.PITCH_MAX_OUTPUT);
        pitchElevator.burnFlash();
    }

    public void NTUpdate() {
        pid.setP(SmartDashboard.getNumber("ELEVATOR_P", 0));
        pid.setI(SmartDashboard.getNumber("ELEVATOR_I", 0));
        pid.setD(SmartDashboard.getNumber("ELEVATOR_D", 0));
        pitchPid.setP(SmartDashboard.getNumber("PITCH_P", 0));
        pitchPid.setI(SmartDashboard.getNumber("PITCH_I", 0));
        pitchPid.setD(SmartDashboard.getNumber("PITCH_D", 0));
    }

    public void stop() {
        rightElevator.stopMotor();
        leftElevator.stopMotor();
        pitchElevator.stopMotor();
    }
}
