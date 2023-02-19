package com.team2502.robot2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2023.Constants;

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
        //TODO CHANGE LIMIT!!!
        liftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);
        leftMotor.setSmartCurrentLimit(40);

        //Sets Left Motor To Spin Opposite Direction To Work With Right Motor
        leftMotor.setInverted(true);
    }

    //Command to Deploy the Intake
    public void deploy(){
        liftMotor.set(Constants.Subsystems.Intake.DEPLOY_SPEED);
    }

    //Command to Retract the Intake
    public void retract(){
        liftMotor.set(-Constants.Subsystems.Intake.DEPLOY_SPEED);
    }

    public void run(double leftSpeed, double rightSpeed) {
        leftMotor.set(-leftSpeed);
        rightMotor.set(-rightSpeed);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
