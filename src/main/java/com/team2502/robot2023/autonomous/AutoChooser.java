package com.team2502.robot2023.autonomous;
import com.team2502.robot2023.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoChooser {
    @FunctionalInterface
    public interface CommandFactory
    {
        CommandBase getInstance(
                DrivetrainSubsystem drivetrainSubsystem,
                IntakeSubsystem intakeSubsystem,
                ArmSubsystem armSubsystem
        );
    }

    /**
     * The actual sendable containing the autonomi
     */
    private static SendableChooser<Autos> autoChooser;

    /**
     * Initialize AutoStartLocationSwitcher#autoChooser, put the enum values in it, and put it on the dashboard
     */
    public static void putToSmartDashboard()
    {
        autoChooser = new SendableChooser<>();

        for (int i = 0; i < Autos.values().length; i++) {
            Autos mode = Autos.values()[i];
            if(i == 0) {
                autoChooser.setDefaultOption(mode.name, mode);
            }
            else {
                autoChooser.addOption(mode.name, mode);
            }
        }

        SmartDashboard.putData("Autonomous Chooser", autoChooser);
    }

    /**
     * Get an instance of the autonomous selected
     *
     * @return A new instance of the selected autonomous
     */
    public static CommandFactory getAutoInstance() { return autoChooser.getSelected().commandFactory; }

    @FunctionalInterface
    private interface SimpleCommandFactory
    {
        CommandBase getInstance();
    }
}
