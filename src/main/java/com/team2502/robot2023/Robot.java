// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2502.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private DigitalInput coastButton;
  private DigitalInput zeroButton;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    LiveWindow.disableAllTelemetry(); // useful for debugging, huge network table/rio load

    coastButton = new DigitalInput(0);
    zeroButton = new DigitalInput(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    double before = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    double elapsed = Timer.getFPGATimestamp() - before;
    SmartDashboard.putNumber("RIO load (ms)", elapsed * 1000);
    SmartDashboard.putNumber("RIO load (%)", elapsed * 1000 / 20 * 100);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
      m_robotContainer.DRIVETRAIN.setPowerNeutralMode(NeutralMode.Brake);
      m_robotContainer.DRIVETRAIN.stop();
  }

  @Override
  public void disabledPeriodic() {
    if (!coastButton.get()) {
      m_robotContainer.DRIVETRAIN.setTurnNeutralMode(NeutralMode.Coast);
      m_robotContainer.ELEVATOR.setAllIdle(CANSparkMax.IdleMode.kCoast);
    } else {
      m_robotContainer.DRIVETRAIN.setTurnNeutralMode(NeutralMode.Brake);
      m_robotContainer.ELEVATOR.setAllIdle(CANSparkMax.IdleMode.kBrake);
    }

    if(!zeroButton.get()) {
      m_robotContainer.ELEVATOR.zeroElevator();
      m_robotContainer.ELEVATOR.zeroArm();
      m_robotContainer.ELEVATOR.zeroPitch();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.DRIVETRAIN.setPowerNeutralMode(NeutralMode.Coast);
    m_robotContainer.DRIVETRAIN.queryStation();
    m_robotContainer.ELEVATOR.retune();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.DRIVETRAIN.setPowerNeutralMode(NeutralMode.Coast);
    m_robotContainer.DRIVETRAIN.queryStation();
    m_robotContainer.ELEVATOR.detune();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
