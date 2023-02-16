package com.team2502.demo2022.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.demo2022.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Manipulator extends SubsystemBase {
    private final CANSparkMax main;

    public Manipulator() {
        main = new CANSparkMax(Constants.HardwareMap.MANIPULATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    }

}