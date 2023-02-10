package com.team2502.demo2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.demo2022.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    //Define motors
    private final CANSparkMax liftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;


    public IntakeSubsystem() {
        // Pulls Motor Info From Constants and Defines Motor Type
        liftMotor = new CANSparkMax(Constants.HardwareMap.LIFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.HardwareMap.RIGHT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.HardwareMap.LEFT_INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Sets Speed Limit To Motors
        //TODO CHANGE SPEED!!!
        liftMotor.setSmartCurrentLimit(20);
        rightMotor.setSmartCurrentLimit(20);
        leftMotor.setSmartCurrentLimit(20);

        //Sets Left Motor To Spin Opposite Direction To Work With Right Motor
        leftMotor.setInverted(true);
    }

    //Command to Deploy and Retract the Intake
    public void deploy(Boolean down){
        if (down == true){
            liftMotor.set(1);
        }else{
            liftMotor.set(-1);
        }
    }

    //Command to Control Intake Speed
    public void intake(double intakeSpeed){
        rightMotor.set(intakeSpeed);
        leftMotor.set(intakeSpeed);
    }
}
