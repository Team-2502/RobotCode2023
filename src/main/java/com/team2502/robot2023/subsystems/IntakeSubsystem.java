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
    private CANSparkMax intake;

    private DigitalInput limitSwitch;

    public IntakeSubsystem() {
        intake = new CANSparkMax(Constants.HardwareMap.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);

        //limitSwitch = new DigitalInput(Constants.HardwareMap.INTAKE_PITCH);

        intake.setSmartCurrentLimit(39);

        setupPid();
    }

    @Override
    public void periodic() {
    }

    private void setupPid() {
    }

    public void setSpeed(double speed) {
        intake.set(speed);
    }

    public boolean isRunning() { return intake.get() != 0; }
}
