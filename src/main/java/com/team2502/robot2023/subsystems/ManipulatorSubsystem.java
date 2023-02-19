package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2023.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
    private CANSparkMax gripper;

    public ManipulatorSubsystem() {
        gripper = new CANSparkMax(Constants.HardwareMap.GRIPPER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void set(double val) {
        gripper.getPIDController().setReference(val, CANSparkMax.ControlType.kSmartMotion);
    }

    public void stop() {
        gripper.stopMotor();
    }
}
