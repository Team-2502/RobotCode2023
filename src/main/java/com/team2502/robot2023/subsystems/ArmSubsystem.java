package com.team2502.robot2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPosition;
import com.team2502.robot2023.Constants.Subsystems.Arm.IntakePosition;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax leftElevator;
    private CANSparkMax rightElevator;
    private CANSparkMax leftPitchElevator;
    private CANSparkMax rightPitchElevator;
    private CANSparkMax pitchIntake;

    private SparkMaxPIDController pid;
    private SparkMaxPIDController pitchPid;
    private SparkMaxPIDController intakePid;

    private DigitalInput limitSwitch;

    public ArmSubsystem() {
        leftElevator = new CANSparkMax(Constants.HardwareMap.LEFT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightElevator = new CANSparkMax(Constants.HardwareMap.RIGHT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftPitchElevator = new CANSparkMax(Constants.HardwareMap.LEFT_PITCH_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightPitchElevator = new CANSparkMax(Constants.HardwareMap.RIGHT_PITCH_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        pitchIntake = new CANSparkMax(Constants.HardwareMap.INTAKE_PITCH, CANSparkMaxLowLevel.MotorType.kBrushless);

        // TODO get followers working
        //leftElevator.burnFlash();
        leftElevator.setIdleMode(IdleMode.kBrake);
        rightElevator.setIdleMode(IdleMode.kBrake);
        leftElevator.follow(rightElevator, true);

        leftElevator.setSmartCurrentLimit(39);
        rightElevator.setSmartCurrentLimit(39);
        leftPitchElevator.setSmartCurrentLimit(39);
        rightElevator.setSmartCurrentLimit(39);
        pitchIntake.setSmartCurrentLimit(39);

        leftPitchElevator.setIdleMode(IdleMode.kBrake);
        rightPitchElevator.setIdleMode(IdleMode.kBrake);
        pitchIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightPitchElevator.follow(leftPitchElevator, true);

        rightElevator.setSoftLimit(SoftLimitDirection.kForward,(float) Constants.Subsystems.Arm.ELEVATOR_LIM_TOP);
        rightElevator.setSoftLimit(SoftLimitDirection.kReverse,(float) Constants.Subsystems.Arm.ELEVATOR_LIM_BOTTOM);
        rightElevator.enableSoftLimit(SoftLimitDirection.kForward,true);
        rightElevator.enableSoftLimit(SoftLimitDirection.kReverse,true);

        limitSwitch = new DigitalInput(Constants.HardwareMap.SWITCH_ELEVATOR);

        pid = rightElevator.getPIDController();
        pitchPid = leftPitchElevator.getPIDController();
        intakePid = pitchIntake.getPIDController();
        setupPID();

        NTInit();setupPID();
        SmartDashboard.putData("Flush Elevator PIDs", new InstantCommand(() -> { NTUpdate();}));

        SmartDashboard.putData("Test: Elvator UP", new InstantCommand(() -> { set(ElevatorPosition.TOP);}));
        SmartDashboard.putData("Test: Elvator DOWN", new InstantCommand(() -> { set(ElevatorPosition.BOTTOM);}));
        SmartDashboard.putData("Test: Wrist OUT", new InstantCommand(() -> { setPitch(ElevatorPitch.OUT);}));
        SmartDashboard.putData("Test: Wrist IN", new InstantCommand(() -> { setPitch(ElevatorPitch.STOWED);}));

        SmartDashboard.putData("Elevator HOME", new InstantCommand(() -> { home();}));
        SmartDashboard.putData("Elevator ZERO", new InstantCommand(() -> { zeroElevator();}));
        SmartDashboard.putData("Pitch ZERO", new InstantCommand(() -> { zeroPitch();}));
        SmartDashboard.putData("Arm ZERO", new InstantCommand(() -> { zeroArm();}));
    }

    @Override
    public void periodic() {
        double elbowAngle = (-leftPitchElevator.getEncoder().getPosition() * Constants.Subsystems.Arm.ELBOW_ROT_TO_DEGREE) + Constants.Subsystems.Arm.ELBOW_ZERO_ANGLE;
        SmartDashboard.putNumber("elbow ang raw", leftPitchElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("elbow ang", elbowAngle);

        double wristAngle = (-pitchIntake.getEncoder().getPosition() * Constants.Subsystems.Arm.WRIST_ROT_TO_DEGREE) + Constants.Subsystems.Arm.WRIST_ZERO_ANGLE;
        SmartDashboard.putNumber("wrist ang soli raw", pitchIntake.getEncoder().getPosition());
        SmartDashboard.putNumber("wrist ang soli", wristAngle);
        wristAngle += elbowAngle;
        SmartDashboard.putNumber("wrist ang raw", pitchIntake.getEncoder().getPosition());
        SmartDashboard.putNumber("wrist ang", wristAngle);

        SmartDashboard.putNumber("elev pos", rightElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("wrist pos", leftPitchElevator.getEncoder().getPosition());
        if (Constants.Subsystems.Arm.NT_TUNE) {
            NTUpdate();
        }
    }

    public void enableSoft(boolean enable) {
        rightElevator.enableSoftLimit(SoftLimitDirection.kForward,enable);
        rightElevator.enableSoftLimit(SoftLimitDirection.kReverse,enable);
    }

    public void setAllIdle(IdleMode im) {
        rightElevator.setIdleMode(im);
        leftElevator.setIdleMode(im);
        leftPitchElevator.setIdleMode(im);
        rightPitchElevator.setIdleMode(im);
        pitchIntake.setIdleMode(im);
    }

    public void detune() {
        pid.setOutputRange(Constants.Subsystems.Arm.ELEVATOR_MIN_OUTPUT_TELEOP, Constants.Subsystems.Arm.ELEVATOR_MAX_OUTPUT_TELEOP);
        pitchPid.setOutputRange(Constants.Subsystems.Arm.PITCH_MIN_OUTPUT_TELEOP, Constants.Subsystems.Arm.PITCH_MAX_OUTPUT_TELEOP);
    }

    public void retune() {
        pid.setOutputRange(Constants.Subsystems.Arm.ELEVATOR_MIN_OUTPUT, Constants.Subsystems.Arm.ELEVATOR_MAX_OUTPUT);
        pitchPid.setOutputRange(Constants.Subsystems.Arm.PITCH_MIN_OUTPUT, Constants.Subsystems.Arm.PITCH_MAX_OUTPUT);
    }

    public void setPitch(ElevatorPitch pitch) {
        leftPitchElevator.getPIDController().setReference(pitch.position, CANSparkMax.ControlType.kPosition);
    }

    public void set(ElevatorPosition pos) {
        rightElevator.getPIDController().setReference(pos.position, CANSparkMax.ControlType.kPosition);
    }

    public void setLinearSpeed(double speed) {
        rightElevator.set(speed);
    }

    public void setPitchSpeed(double speed) {
        leftPitchElevator.set(speed);
        //rightPitchElevator.set(-speed);
    }

    public double getLinear() {
        return rightElevator.getEncoder().getPosition();
    }

    public double getPitch() {
        return leftPitchElevator.getEncoder().getPosition();
    }

    /** @return is it safe to unstow the arm */
    public boolean safePitch() {
        if (rightElevator.getEncoder().getPosition() < ElevatorPosition.SAFE_PITCH.position) {
            return true;
        } else {
            return false;
        }
    }

    public void setArmWrist(IntakePosition position) {
        pitchIntake.getPIDController().setReference(position.pitchWrist, CANSparkMax.ControlType.kPosition);
    }

    public void setArmPitch(IntakePosition position) {
        leftPitchElevator.getPIDController().setReference(position.pitchElbow, CANSparkMax.ControlType.kPosition);
    }

    public void setArmPitchSpeed(double speed) {
        pitchIntake.set(speed);
    }

    public void homeArm() {
        while (!limitSwitch.get()) {
            setPitchSpeed(0.1);
            if (pitchIntake.getOutputCurrent() > 15) {
                break;
            }
        }

        pitchIntake.getEncoder().setPosition(0);
        setArmPitch(IntakePosition.IN);
    }

    public void zeroArm() {
        pitchIntake.getEncoder().setPosition(0);
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

    public void zeroPitch() {
        leftPitchElevator.getEncoder().setPosition(0);
    }

    public void zeroElevator() {
        rightElevator.getEncoder().setPosition(0);
    }

    private void NTInit() {
        SmartDashboard.putNumber("ELEVATOR_P", Constants.Subsystems.Arm.ELEVATOR_P);
        SmartDashboard.putNumber("ELEVATOR_I", Constants.Subsystems.Arm.ELEVATOR_I);
        SmartDashboard.putNumber("ELEVATOR_D", Constants.Subsystems.Arm.ELEVATOR_D);
        SmartDashboard.putNumber("PITCH_P", Constants.Subsystems.Arm.PITCH_P);
        SmartDashboard.putNumber("PITCH_I", Constants.Subsystems.Arm.PITCH_I);
        SmartDashboard.putNumber("PITCH_D", Constants.Subsystems.Arm.PITCH_D);
    }

    private void setupPID() {
        pid.setP(Constants.Subsystems.Arm.ELEVATOR_P);
        pid.setI(Constants.Subsystems.Arm.ELEVATOR_I);
        pid.setD(Constants.Subsystems.Arm.ELEVATOR_D);
        pid.setOutputRange(Constants.Subsystems.Arm.ELEVATOR_MIN_OUTPUT, Constants.Subsystems.Arm.ELEVATOR_MAX_OUTPUT);
        rightElevator.burnFlash();

        pitchPid.setP(Constants.Subsystems.Arm.PITCH_P);
        pitchPid.setI(Constants.Subsystems.Arm.PITCH_I);
        pitchPid.setD(Constants.Subsystems.Arm.PITCH_D);
        pitchPid.setOutputRange(Constants.Subsystems.Arm.PITCH_MIN_OUTPUT, Constants.Subsystems.Arm.PITCH_MAX_OUTPUT);
        leftPitchElevator.burnFlash();

        intakePid.setP(Constants.Subsystems.Arm.INTAKE_P);
        intakePid.setI(Constants.Subsystems.Arm.INTAKE_I);
        intakePid.setD(Constants.Subsystems.Arm.INTAKE_D);
        pitchPid.setOutputRange(Constants.Subsystems.Arm.INTAKE_MIN_OUTPUT, Constants.Subsystems.Arm.INTAKE_MAX_OUTPUT);
        pitchIntake.burnFlash();
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
        leftPitchElevator.stopMotor();
        rightPitchElevator.stopMotor();
    }
}
