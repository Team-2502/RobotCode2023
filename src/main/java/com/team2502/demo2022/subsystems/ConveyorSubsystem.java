package com.team2502.demo2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.demo2022.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
    private CANSparkMax leftie;
    private CANSparkMax rightie;

    public ConveyorSubsystem() {
        leftie = new CANSparkMax(Constants.HardwareMap.LEFT_CONVEYOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightie = new CANSparkMax(Constants.HardwareMap.RIGHT_CONVEYOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void runConveyor(double speed){
        leftie.set(speed);
        rightie.set(-speed);
    }
}
