package com.team2502.robot2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.unmanaged.UnmanagedJNI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2023.Utils;
import com.team2502.robot2023.Constants.HardwareMap;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain;

import java.lang.Math;

//import com.team2502.robot2022.Constants.Subsystem.Drivetrain;

public class DrivetrainSubsystem extends SubsystemBase{
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    private WPI_TalonFX drivetrainPowerBackLeft;
    private WPI_TalonFX drivetrainPowerFrontLeft;
    private WPI_TalonFX drivetrainPowerBackRight;
    private WPI_TalonFX drivetrainPowerFrontRight;

    private WPI_TalonFX drivetrainTurnBackLeft;
    private WPI_TalonFX drivetrainTurnFrontLeft;
    private WPI_TalonFX drivetrainTurnBackRight;
    private WPI_TalonFX drivetrainTurnFrontRight;

    /*private CANSparkMax drivetrainEncoderBackLeft;
    private CANSparkMax drivetrainEncoderFrontLeft;
    private CANSparkMax drivetrainEncoderBackRight;
    private CANSparkMax drivetrainEncoderFrontRight;*/

    private CANCoder drivetrainEncoderBackLeft;
    private CANCoder drivetrainEncoderFrontLeft;
    private CANCoder drivetrainEncoderBackRight;
    private CANCoder drivetrainEncoderFrontRight;

    private Field2d field;

    private AHRS navX = new AHRS();

    private Solenoid solenoid;

    private double currentPos;

    public DrivetrainSubsystem(){
        PhotonVisionSubsystem vision = new PhotonVisionSubsystem();

        drivetrainPowerBackLeft = new WPI_TalonFX(HardwareMap.BL_DRIVE_MOTOR, "can0");
        drivetrainPowerFrontLeft = new WPI_TalonFX(HardwareMap.FL_DRIVE_MOTOR, "can0");
        drivetrainPowerFrontRight = new WPI_TalonFX(HardwareMap.FR_DRIVE_MOTOR, "can0");
        drivetrainPowerBackRight = new WPI_TalonFX(HardwareMap.BR_DRIVE_MOTOR, "can0");
        
        drivetrainTurnBackLeft = new WPI_TalonFX(HardwareMap.BL_TURN_MOTOR, "can0");
        drivetrainTurnFrontLeft = new WPI_TalonFX(HardwareMap.FL_TURN_MOTOR, "can0");
        drivetrainTurnFrontRight = new WPI_TalonFX(HardwareMap.FR_TURN_MOTOR, "can0");
        drivetrainTurnBackRight = new WPI_TalonFX(HardwareMap.BR_TURN_MOTOR, "can0");

        /*drivetrainEncoderBackLeft = new CANSparkMax(HardwareMap.BL_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);
        drivetrainEncoderFrontLeft = new CANSparkMax(HardwareMap.FL_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);
        drivetrainEncoderBackRight = new CANSparkMax(HardwareMap.BR_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);
        drivetrainEncoderFrontRight = new CANSparkMax(HardwareMap.FR_TURN_ENCODER, CANSparkMaxLowLevel.MotorType.kBrushless);*/

        drivetrainEncoderBackLeft = new CANCoder(HardwareMap.BL_TURN_ENCODER, "can0");
        drivetrainEncoderFrontLeft = new CANCoder(HardwareMap.FL_TURN_ENCODER, "can0");
        drivetrainEncoderFrontRight = new CANCoder(HardwareMap.FR_TURN_ENCODER, "can0");
        drivetrainEncoderBackRight = new CANCoder(HardwareMap.BR_TURN_ENCODER, "can0");

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

        Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-getHeading()), getModulePositions(), startPose2d);

        field = new Field2d();
        //field.initSendable(builder);


        //drivetrainBackRight.setInverted(TalonFXInvertType.CounterClockwise);
        //drivetrainBackLeft.setInverted(TalonFXInvertType.CounterClockwise);
        //drivetrainFrontRight.setNeutralMode(NeutralMode.Brake);
        //drivetrainFrontLeft.setNeutralMode(NeutralMode.Brake);
        //drive = new DifferentialDrive(drivetrainFrontLeft, drivetrainFrontRight);

        //drive.setSafetyEnabled(false); // suppress "watchdog not fed" errors
    }

    public SwerveModulePosition[] getModulePositions() {
        Rotation2d FLRotation = Rotation2d.fromDegrees(
                drivetrainEncoderFrontLeft.getPosition()
        );
        Rotation2d FRRotation = Rotation2d.fromDegrees(
                drivetrainEncoderFrontLeft.getPosition()
        );
        Rotation2d BLRotation = Rotation2d.fromDegrees(
                drivetrainEncoderBackLeft.getPosition()
        );
        Rotation2d BRRotation = Rotation2d.fromDegrees(
                drivetrainEncoderBackRight.getPosition()
        );

        SwerveModulePosition FRPosition = new SwerveModulePosition(
                drivetrainPowerFrontRight.getSelectedSensorPosition()*Drivetrain.SWERVE_FALCON_METERS_PER_TICK, 
                FRRotation);
        SwerveModulePosition FLPosition = new SwerveModulePosition(
                drivetrainPowerFrontLeft.getSelectedSensorPosition()*Drivetrain.SWERVE_FALCON_METERS_PER_TICK, 
                FLRotation);
        SwerveModulePosition BRPosition = new SwerveModulePosition(
                drivetrainPowerBackRight.getSelectedSensorPosition()*Drivetrain.SWERVE_FALCON_METERS_PER_TICK, 
                BRRotation);
        SwerveModulePosition BLPosition = new SwerveModulePosition(
                drivetrainPowerBackLeft.getSelectedSensorPosition()*Drivetrain.SWERVE_FALCON_METERS_PER_TICK, 
                BLRotation);

        SwerveModulePosition[] modulePositions = {FLPosition,FRPosition,BLPosition,BRPosition};

        return modulePositions;

    }
    public void setSpeeds(ChassisSpeeds speed, Translation2d centerOfRotation) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed, centerOfRotation);

        Rotation2d FLRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderFrontLeft.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnFrontLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                drivetrainEncoderFrontLeft.getPosition()
        );
        Rotation2d FRRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderFrontRight.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnFrontRight.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                drivetrainEncoderFrontLeft.getPosition()
        );
        Rotation2d BLRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderBackLeft.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnBackLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                drivetrainEncoderBackLeft.getPosition()
        );
        Rotation2d BRRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderBackRight.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnBackRight.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                drivetrainEncoderBackRight.getPosition()
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
    public double getInchesTraveled() {
	    return (drivetrainPowerFrontLeft.getSelectedSensorPosition() / Drivetrain.SWERVE_FALCON_TICKS_PER_INCH) / 1000;
    }

    public double getMetersTraveled() {
	    return (drivetrainPowerFrontLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_METERS_PER_TICK);
    }

    public void setToDistance(double distance) {
        currentPos = drivetrainPowerFrontLeft.getSelectedSensorPosition();

        drivetrainPowerFrontLeft.set(ControlMode.Position, distance + currentPos);
        drivetrainPowerFrontRight.set(ControlMode.Position, distance + currentPos);
        drivetrainPowerBackLeft.set(ControlMode.Position, distance + currentPos);
        drivetrainPowerBackRight.set(ControlMode.Position, distance + currentPos);
    }

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

    public double getRpm() {
        return (drivetrainPowerFrontLeft.getSelectedSensorVelocity() / Drivetrain.SWERVE_FALCON_TICKS_PER_INCH);
    }
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

    public double getMaxTemp() {
        double fl = drivetrainPowerFrontLeft.getTemperature();
        double fr = drivetrainPowerFrontRight.getTemperature();
        double bl = drivetrainPowerFrontLeft.getTemperature();
        double br = drivetrainPowerBackRight.getTemperature();

        return Math.max(Math.max(fl, fr), Math.max(bl, br));
    }

    @Override
    public void periodic(){
        odometry.update(Rotation2d.fromDegrees(-getHeading()), getModulePositions());

        double[] position = {odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY()};
        SmartDashboard.putNumberArray("Position", position);
        
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("field", field);

        SmartDashboard.putNumber("Angle", navX.getAngle());
        //SmartDashboard.putNumber("RPM", getRpm());
	    SmartDashboard.putNumber("fr temp", drivetrainPowerFrontRight.getTemperature());
        //SmartDashboard.putBoolean("High Gear", getGear());
        //SmartDashboard.putNumber("FL rotation", drivetrainTurnFrontLeft.getSelectedSensorPosition()/360);
        //SmartDashboard.putNumber("FR Rotation", drivetrainTurnFrontRight.getSelectedSensorPosition()/360);
        SmartDashboard.putNumber("Avg Drivetrain Temp", getAverageTemp());
        SmartDashboard.putNumber("Inches Travled", getInchesTraveled());
        SmartDashboard.putNumber("Meters Travled", getMetersTraveled());
        SmartDashboard.putNumber("Turning Error", getHeading() + Rotation2d.fromDegrees(2).getDegrees());
    }
}
