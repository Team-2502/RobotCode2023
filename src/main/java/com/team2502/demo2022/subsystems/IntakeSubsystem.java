package com.team2502.demo2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.demo2022.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax liftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;


    public IntakeSubsystem() {
        liftMotor = new CANSparkMax(Constants.HardwareMap.LIFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.HardwareMap.RIGHT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.HardwareMap.LEFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
}
