package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2023.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
    private CANSparkMax conveyor;

    public ConveyorSubsystem() {
        conveyor = new CANSparkMax(Constants.HardwareMap.CONVEYOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        conveyor.setSmartCurrentLimit(30);
    }

    public void run(double speed){
        conveyor.set(speed);
    }

    public void stop() {
        conveyor.stopMotor();
    }
}
