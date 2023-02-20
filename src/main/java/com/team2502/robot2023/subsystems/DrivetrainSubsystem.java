package com.team2502.robot2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2502.robot2023.Utils;
import com.team2502.robot2023.Constants.HardwareMap;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain;
import com.team2502.robot2023.Constants.Subsystems.Drivetrain.*;

import java.lang.Math;


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

    private double currentPos;

    public double fieldOrientedOffset; // gyro offset to driver

    private enum ControlModes {
        POSE, /// approach the given absolute x,y,theta pose
        VELOCITY, /// attempt to reach the given x,y,theta velocity
    }

    private ControlModes controlMode; // current control strategy
    // pose control constants
    private PIDController xPidController;
    private PIDController yPidController;
    private PIDController rPidController;

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

        drivetrainEncoderBackLeft = new CANCoder(HardwareMap.BL_TURN_ENCODER, "can0");
        drivetrainEncoderFrontLeft = new CANCoder(HardwareMap.FL_TURN_ENCODER, "can0");
        drivetrainEncoderFrontRight = new CANCoder(HardwareMap.FR_TURN_ENCODER, "can0");
        drivetrainEncoderBackRight = new CANCoder(HardwareMap.BR_TURN_ENCODER, "can0");

        Translation2d m_frontLeftLocation = new Translation2d(Drivetrain.SWERVE_WIDTH, -Drivetrain.SWERVE_LENGTH);
        Translation2d m_frontRightLocation = new Translation2d(Drivetrain.SWERVE_WIDTH, Drivetrain.SWERVE_LENGTH);
        Translation2d m_backLeftLocation = new Translation2d(-Drivetrain.SWERVE_WIDTH, -Drivetrain.SWERVE_LENGTH);
        Translation2d m_backRightLocation = new Translation2d(-Drivetrain.SWERVE_WIDTH, Drivetrain.SWERVE_LENGTH);

        kinematics = new SwerveDriveKinematics(
                m_frontLeftLocation,
                m_frontRightLocation,
                m_backLeftLocation,
                m_backRightLocation
        );

        Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), startPose2d);

        field = new Field2d();

        controlMode = ControlModes.VELOCITY;

        // initialize PIDs
        this.xPidController = new PIDController(Drivetrain.DRIVETRAIN_MOVE_P, Drivetrain.DRIVETRAIN_MOVE_I, Drivetrain.DRIVETRAIN_MOVE_D);
        this.yPidController = new PIDController(Drivetrain.DRIVETRAIN_MOVE_P, Drivetrain.DRIVETRAIN_MOVE_I, Drivetrain.DRIVETRAIN_MOVE_D);
        this.rPidController = new PIDController(Drivetrain.DRIVETRAIN_TURN_P, Drivetrain.DRIVETRAIN_TURN_I, Drivetrain.DRIVETRAIN_TURN_D);

        fieldOrientedOffset = 0;
    }

    private SwerveModulePosition[] getModulePositions() {
        Rotation2d FLRotation = Rotation2d.fromDegrees(
                -drivetrainEncoderFrontLeft.getPosition()
        );
        Rotation2d FRRotation = Rotation2d.fromDegrees(
                -drivetrainEncoderFrontRight.getPosition()
        );
        Rotation2d BLRotation = Rotation2d.fromDegrees(
                -drivetrainEncoderBackLeft.getPosition()
        );
        Rotation2d BRRotation = Rotation2d.fromDegrees(
                -drivetrainEncoderBackRight.getPosition()
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

    /** setGoalPose
     * sets the target position and rotation for the robot, using the internal PID controllers
     * @param pose target pose
     */
    public void setGoalPose(Pose2d pose) {
        controlMode = ControlModes.POSE;

        this.xPidController.setSetpoint(pose.getX());
        this.yPidController.setSetpoint(pose.getY());
        this.rPidController.setSetpoint(pose.getRotation().getRadians());
    }

    /** atGoalPose
     * @return are all movement PIDs settled?
     */
    public boolean atGoalPose() {
        switch (controlMode) {
            case POSE:
                return xPidController.atSetpoint() && yPidController.atSetpoint() && rPidController.atSetpoint();
            case VELOCITY:
                return false;
        }
        return false;
    }


	/** setSpeeds
     * sets a velocity for the swerve drive
	 * @param speed x,y,theta goal velocity (m/s)
	 */
    public void setSpeeds(ChassisSpeeds speed) {
        Translation2d centerOfRotation = new Translation2d(0, 0);
        setSpeeds(speed, centerOfRotation);
    }

	/** setSpeeds
     * sets a velocity for the swerve drive
	 * @param speed x,y,theta goal velocity (m/s)
	 * @param centerOfRotation x,y position (m) on the chassis to rotate around
	 */
    public void setSpeeds(ChassisSpeeds speed, Translation2d centerOfRotation) {
        controlMode = ControlModes.VELOCITY;

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed, centerOfRotation);

        Rotation2d FLRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderFrontLeft.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnFrontLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                -drivetrainEncoderFrontLeft.getPosition()
        );
        Rotation2d FRRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderFrontRight.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnFrontRight.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                -drivetrainEncoderFrontRight.getPosition()
        );
        Rotation2d BLRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderBackLeft.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnBackLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                -drivetrainEncoderBackLeft.getPosition()
        );
        Rotation2d BRRotation = Rotation2d.fromDegrees(
            //drivetrainEncoderBackRight.getAlternateEncoder(Drivetrain.SWERVE_ENCODER_COUNTS_PER_REV).getPosition()/360
            //drivetrainTurnBackRight.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES
                -drivetrainEncoderBackRight.getPosition()
        );
        
        SwerveModuleState FLState = SwerveModuleState.optimize(moduleStates[0], FLRotation);
        SwerveModuleState FRState = SwerveModuleState.optimize(moduleStates[1], FRRotation);
        SwerveModuleState BLState = SwerveModuleState.optimize(moduleStates[2], BLRotation);
        SwerveModuleState BRState = SwerveModuleState.optimize(moduleStates[3], BRRotation);

        drivetrainTurnFrontLeft.set(ControlMode.Position, Utils.placeInAppropriate0To360Scope(FLRotation.getDegrees(), FLState.angle.getDegrees()) / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);
        drivetrainTurnFrontRight.set(ControlMode.Position, Utils.placeInAppropriate0To360Scope(FRRotation.getDegrees(), FRState.angle.getDegrees()) / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);
        drivetrainTurnBackLeft.set(ControlMode.Position, Utils.placeInAppropriate0To360Scope(BLRotation.getDegrees(), BLState.angle.getDegrees()) / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);
        drivetrainTurnBackRight.set(ControlMode.Position, Utils.placeInAppropriate0To360Scope(BRRotation.getDegrees(), BRState.angle.getDegrees()) / Drivetrain.SWERVE_FALCON_ENCODER_COUNTS_TO_DEGREES);


        drivetrainPowerFrontLeft.set(ControlMode.Velocity, FLState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        drivetrainPowerFrontRight.set(ControlMode.Velocity, FRState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        drivetrainPowerBackLeft.set(ControlMode.Velocity, BLState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);
        drivetrainPowerBackRight.set(ControlMode.Velocity, BRState.speedMetersPerSecond * Drivetrain.SWERVE_METERS_PER_SECOND_TO_CTRE);

        SmartDashboard.putNumber("FLmps ", FLState.speedMetersPerSecond);
        SmartDashboard.putNumber("FLTang", FLState.angle.getDegrees());
        SmartDashboard.putNumber("FL Angle", FLRotation.getDegrees());
    }

    /** stop drivetrain by freezing all motors */
    public void stop() {
        controlMode = ControlModes.VELOCITY;
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
//    * inches since init
//    * @return inches since initialization
//     */
    public double getInchesTraveled() {
	    return (drivetrainPowerFrontLeft.getSelectedSensorPosition() / Drivetrain.SWERVE_FALCON_TICKS_PER_INCH) / 1000;
    }

    public double getMetersTraveled() {
	    return (drivetrainPowerFrontLeft.getSelectedSensorPosition() * Drivetrain.SWERVE_FALCON_METERS_PER_TICK);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
    
    public void resetHeading() {
        navX.reset();
    }

    public void resetOffset() {
        fieldOrientedOffset = -navX.getAngle();
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
        Pose2d pose = odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());
        

        SmartDashboard.putBoolean("GTA en?", controlMode == ControlModes.POSE);

        switch (controlMode) {
            case VELOCITY:
                controlMode = ControlModes.VELOCITY;
                break;
            case POSE:
                double xPower = xPidController.calculate(pose.getX());
                double yPower = yPidController.calculate(pose.getY());
                //double rPower = rPidController.calculate(pose.getRotation().getRadians());
                double rPower = -rPidController.calculate(Units.degreesToRadians(getHeading()));

                SmartDashboard.putNumber("GTA xp", xPower);
                SmartDashboard.putNumber("GTA yp", yPower);
                SmartDashboard.putNumber("GTA rp", rPower);

                ChassisSpeeds speeds = new ChassisSpeeds(xPower, yPower, rPower);
                setSpeeds(speeds);

                controlMode = ControlModes.POSE;
                break;
        }

        // telemetry
        double[] position = {odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY()};
        SmartDashboard.putNumberArray("Position", position);
        
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("field", field);

        SmartDashboard.putNumber("Angle", navX.getAngle());
	    SmartDashboard.putNumber("fr temp", drivetrainPowerFrontRight.getTemperature());
        SmartDashboard.putNumber("Avg Drivetrain Temp", getAverageTemp());
        SmartDashboard.putNumber("Meters Traveled", getMetersTraveled());
        SmartDashboard.putNumber("Turning Error", getHeading() + Rotation2d.fromDegrees(2).getDegrees());
    }
}
