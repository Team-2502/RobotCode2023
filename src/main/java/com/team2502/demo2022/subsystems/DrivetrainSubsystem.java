package com.team2502.demo2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.unmanaged.UnmanagedJNI;
import com.kauailabs.navx.frc.AHRS;
import com.team2502.demo2022.Constants.HardwareMap;
import com.team2502.demo2022.Constants.Subsystems.Drivetrain;
import com.team2502.demo2022.Utils;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import java.lang.Math;

//import com.team2502.robot2022.Constants.Subsystem.Drivetrain;

public class DrivetrainSubsystem extends SubsystemBase{
    private SwerveDriveKinematics kinematics;

    private WPI_TalonFX drivetrainPowerBackLeft;
    private WPI_TalonFX drivetrainPowerFrontLeft;
    private WPI_TalonFX drivetrainPowerBackRight;
    private WPI_TalonFX drivetrainPowerFrontRight;

    private WPI_TalonFX drivetrainTurnBackLeft;
    private WPI_TalonFX drivetrainTurnFrontLeft;
    private WPI_TalonFX drivetrainTurnBackRight;
    private WPI_TalonFX drivetrainTurnFrontRight;

    private CANSparkMax drivetrainEncoderBackLeft;
    private CANSparkMax drivetrainEncoderFrontLeft;
    private CANSparkMax drivetrainEncoderBackRight;
    private CANSparkMax drivetrainEncoderFrontRight;

    private AHRS navX = new AHRS();

    private Solenoid solenoid;

    public DrivetrainSubsystem(){
        drivetrainPowerBackLeft = new WPI_TalonFX(HardwareMap.BL_DRIVE_MOTOR);
        drivetrainPowerFrontLeft = new WPI_TalonFX(HardwareMap.FL_DRIVE_MOTOR);
        drivetrainPowerFrontRight = new WPI_TalonFX(HardwareMap.FR_DRIVE_MOTOR);
        drivetrainPowerBackRight = new WPI_TalonFX(HardwareMap.BR_DRIVE_MOTOR);
        
        drivetrainTurnBackLeft = new WPI_TalonFX(HardwareMap.BL_TURN_MOTOR);
        drivetrainTurnFrontLeft = new WPI_TalonFX(HardwareMap.FL_TURN_MOTOR);
        drivetrainTurnFrontRight = new WPI_TalonFX(HardwareMap.FR_TURN_MOTOR);
        drivetrainTurnBackRight = new WPI_TalonFX(HardwareMap.BR_TURN_MOTOR);

        drivetrainEncoderBackLeft = new CANSparkMax(HardwareMap.BL_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);
        drivetrainEncoderFrontLeft = new CANSparkMax(HardwareMap.FL_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);
        drivetrainEncoderBackRight = new CANSparkMax(HardwareMap.BR_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);
        drivetrainEncoderFrontRight = new CANSparkMax(HardwareMap.FR_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);

        Translation2d m_frontLeftLocation = new Translation2d(Drivetrain.SWERVE_WIDTH, Drivetrain.SWERVE_LENGTH);
        Translation2d m_frontRightLocation = new Translation2d(Drivetrain.SWERVE_WIDTH, -Drivetrain.SWERVE_LENGTH);
        Translation2d m_backLeftLocation = new Translation2d(-Drivetrain.SWERVE_WIDTH, Drivetrain.SWERVE_LENGTH);
        Translation2d m_backRightLocation = new Translation2d(-Drivetrain.SWERVE_WIDTH, -Drivetrain.SWERVE_LENGTH);

        kinematics = new SwerveDriveKinematics(
                m_frontLeftLocation,
                m_frontRightLocation,
                m_backLeftLocation,
                m_backRightLocation
        );


        //drivetrainBackRight.setInverted(TalonFXInvertType.CounterClockwise);
        //drivetrainBackLeft.setInverted(TalonFXInvertType.CounterClockwise);
        //drivetrainFrontRight.setNeutralMode(NeutralMode.Brake);
        //drivetrainFrontLeft.setNeutralMode(NeutralMode.Brake);
        //drive = new DifferentialDrive(drivetrainFrontLeft, drivetrainFrontRight);

        //drive.setSafetyEnabled(false); // suppress "watchdog not fed" errors
    }

    public void setSpeeds(ChassisSpeeds speed, Translation2d centerOfRotation) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed, centerOfRotation);

        Rotation2d FLRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderFrontLeft.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            drivetrainTurnFrontLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
        );
        Rotation2d FRRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderFrontRight.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            drivetrainTurnFrontRight.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
        );
        Rotation2d BLRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderBackLeft.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            drivetrainTurnBackLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
        );
        Rotation2d BRRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderBackRight.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            drivetrainTurnBackRight.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
        );
        
        SwerveModuleState FLState =
            Utils.optimize(
                moduleStates[0], 
                    FLRotation
            );
        SwerveModuleState FRState =
                Utils.optimize(
                moduleStates[1],
                    FRRotation
            );
        SwerveModuleState BLState =
                Utils.optimize(
                moduleStates[2],
                    BLRotation
            );
        SwerveModuleState BRState =
                Utils.optimize(
                moduleStates[3],
                    BRRotation
            );

        drivetrainTurnFrontLeft.set(ControlMode.Position, FLState.angle.getDegrees() / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);
        drivetrainTurnFrontRight.set(ControlMode.Position, FRState.angle.getDegrees() / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);
        drivetrainTurnBackLeft.set(ControlMode.Position, BLState.angle.getDegrees() / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);
        drivetrainTurnBackRight.set(ControlMode.Position, BRState.angle.getDegrees() / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);


        drivetrainPowerFrontLeft.set(ControlMode.Velocity, FLState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        drivetrainPowerFrontRight.set(ControlMode.Velocity, FRState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        drivetrainPowerBackLeft.set(ControlMode.Velocity, BLState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        drivetrainPowerBackRight.set(ControlMode.Velocity, BRState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);

        SmartDashboard.putNumber("FLmps ", FLState.speedMetersPerSecond);
        SmartDashboard.putNumber("FLCTRUNITS ", FLState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        SmartDashboard.putNumber("FLTang", FLState.angle.getDegrees());
        SmartDashboard.putNumber("FL Angle", FLRotation.getDegrees());
    }

    public void stop() {
        drivetrainPowerBackLeft.stopMotor();
        drivetrainPowerBackRight.stopMotor();
        drivetrainPowerFrontLeft.stopMotor();
        drivetrainPowerFrontRight.stopMotor();

        drivetrainTurnBackLeft.stopMotor();
        drivetrainTurnBackRight.stopMotor();
        drivetrainTurnFrontLeft.stopMotor();
        drivetrainTurnFrontRight.stopMotor();
    }

//    /**
//    * Average drivetrain motor revs
//    * @return double revs since restart
//     */
//    public double getRevsAvg() {
//	    return (
//			    -drivetrainFrontRight.getSelectedSensorPosition()+
//			    drivetrainFrontLeft.getSelectedSensorPosition()
//		   )/2;
//    }
//
//    /**
//    * inches since init
//    * @return inches since initialization
//     */
//    public double getInchesTraveled() {
//	    /*return (getRevsAvg()/2048) // encoder
//		    * (24 / 50) // gearbox
//		    * (6 * Math.PI); // wheel radius */
//	    return drivetrainFrontRight.getSelectedSensorPosition() / Drivetrain.TICKS_PER_INCH;
//    }

    public void setPowerNeutralMode(NeutralMode nm) {
        drivetrainPowerBackLeft.setNeutralMode(nm);
        drivetrainPowerBackRight.setNeutralMode(nm);
        drivetrainPowerFrontLeft.setNeutralMode(nm);
        drivetrainPowerFrontRight.stopMotor();
    }

    public void setTurnNeutralMode(NeutralMode nm) {
        drivetrainTurnBackLeft.setNeutralMode(nm);
        drivetrainTurnBackRight.setNeutralMode(nm);
        drivetrainTurnFrontLeft.setNeutralMode(nm);
        drivetrainTurnFrontRight.setNeutralMode(nm);
    }

//    public double getRpm() {
//        return (drivetrainFrontLeft.getSelectedSensorVelocity() / Drivetrain.TICKS_PER_INCH);
//    }
//
//    public double getHeading()
//    {
//        return Math.IEEEremainder(-navX.getAngle(), 360D);
//    }
//
    public void resetHeading() {
        navX.reset();
    }

    public double getHeading() {
        return navX.getAngle();
    }

    public double getAverageTemp() {
        double fl = drivetrainPowerFrontLeft.getTemperature();
        double fr = drivetrainPowerFrontRight.getTemperature();
        double bl = drivetrainPowerFrontLeft.getTemperature();
        double br = drivetrainPowerBackRight.getTemperature();

        return (fl + fr + bl + br) / 4;
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Angle", navX.getAngle());
        //SmartDashboard.putNumber("RPM", getRpm());
	    SmartDashboard.putNumber("fr temp", drivetrainPowerFrontRight.getTemperature());
        //SmartDashboard.putBoolean("High Gear", getGear());
        //SmartDashboard.putNumber("FL rotation", drivetrainTurnFrontLeft.getSelectedSensorPosition()/360);
        //SmartDashboard.putNumber("FR Rotation", drivetrainTurnFrontRight.getSelectedSensorPosition()/360);
        SmartDashboard.putNumber("Avg Drivetrain Temp", getAverageTemp());
    }
}
