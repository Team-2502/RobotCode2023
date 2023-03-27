package com.team2502.robot2023.autonomous;

import com.team2502.robot2023.Constants.Subsystems.Arm.*;
import com.team2502.robot2023.commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPosition;
import com.team2502.robot2023.autonomous.AutoChooser.CommandFactory;

/**
 * class for autonomous command groups
 * put new groups before the do nothing group
 * */
public enum Autos { // first auto is default
        ENGAGE((d,i,a) -> Commands.sequence( 
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            new InstantCommand(d::resetRoll),
            Commands.deadline(Commands.waitSeconds(1.65), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
            Commands.deadline(new TimeLeftCommand(1), new BalanceCommand(d, false)),
            Commands.deadline(Commands.waitSeconds(.5), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        LEAVE_COMMUNITY("Leave CT spawn", (d,i,a) -> Commands.sequence(
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
        )),

        PLACE_CUBE_GRAB_CUBE("Place and grab cube", (d,i,e) -> Commands.sequence(
                new InstantCommand(d::resetPitch),
                new InstantCommand(d::resetHeading),
                new InstantCommand(() -> d.setPose(new Pose2d(1.75, 4.45, Rotation2d.fromDegrees(0))), d),
                new InstantCommand(() -> i.setSpeed(0.25)),
                Commands.deadline(Commands.waitSeconds(2.25), new SetArmSimpleCommand(e, ElevatorPosition.CUBE_TOP, IntakePosition.CUBE_TOP)),
                Commands.deadline(Commands.waitSeconds(0.125), new InstantCommand(() -> i.setSpeed(-0.25))),
                Commands.deadline(new InstantCommand(() -> i.setSpeed(0))),
                Commands.deadline(Commands.waitSeconds(1.5), new SetArmSimpleCommand(e, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND)),
                Commands.deadline(new InstantCommand(() -> i.setSpeed(0.5))),
                Commands.deadline(Commands.waitSeconds(8), new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/blue-score-pickup"), new SetArmSimpleCommand(e, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND)),
                new InstantCommand(() -> i.setSpeed(0.25)),
                Commands.deadline(Commands.waitSeconds(2), new SetArmSimpleCommand(e, ElevatorPosition.CUBE_MID, IntakePosition.CUBE_MID)),
                Commands.deadline(Commands.waitSeconds(0.125), new InstantCommand(() -> i.setSpeed(-0.25))),
                new InstantCommand(() -> i.setSpeed(0))
        )),

        ONE_CUBE_MID_LEAVE_ENGAGE_UNIHETERO((d,i,a) -> Commands.sequence( // score one cube mid, leave community and balance
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            Commands.deadline(Commands.waitSeconds(2.25), new SetArmSimpleCommand(a, ElevatorPosition.CUBE_TOP, IntakePosition.CUBE_TOP)),
            Commands.deadline(Commands.waitSeconds(0.125), new InstantCommand(() -> i.setSpeed(-0.25))),
            Commands.deadline(new InstantCommand(() -> i.setSpeed(0))),
            Commands.deadline(Commands.waitSeconds(0.6), new SetArmSimpleCommand(a, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND)),
            Commands.deadline(Commands.waitSeconds(1.5), new SetArmSimpleCommand(a, ElevatorPosition.BOTTOM, IntakePosition.INIT)),
            Commands.deadline(Commands.waitSeconds(0.37), new YawLockedTranspose(d, new ChassisSpeeds(-.8,0,0), false)),
            Commands.deadline(Commands.waitSeconds(0.5), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0), false)),
            Commands.deadline(new TimeLeftCommand(0.75), new BalanceCommand(d, false)),
            Commands.deadline(Commands.waitSeconds(.25), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0), false)),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),


        PLACE_CUBE_AND_BALANCE("Place and balance left", (d,i,e) -> Commands.sequence(
                new InstantCommand(d::resetPitch),
                new InstantCommand(d::resetHeading),
                new InstantCommand(() -> d.setPose(new Pose2d(1.75, 4.45, Rotation2d.fromDegrees(0))), d),
                new InstantCommand(() -> i.setSpeed(0.25)),
                Commands.deadline(Commands.waitSeconds(2.25), new SetArmSimpleCommand(e, ElevatorPosition.CUBE_TOP, IntakePosition.CUBE_TOP)),
                Commands.deadline(Commands.waitSeconds(0.125), new InstantCommand(() -> i.setSpeed(-0.25))),
                Commands.deadline(new InstantCommand(() -> i.setSpeed(0))),
                Commands.deadline(Commands.waitSeconds(1.5), new SetArmSimpleCommand(e, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND)),
                Commands.deadline(Commands.waitSeconds(3), new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/blue-score-balance-left")),
                Commands.deadline(Commands.waitSeconds(0.5), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
                Commands.deadline(Commands.waitSeconds(8), new BalanceCommand(d, false)),
                //Commands.deadline(Commands.waitSeconds(.5), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0))),
                new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        PLACE_CUBE_SHOOT_CUBE_BALANCE("Place, shoot, balance", (d,i,e) -> Commands.sequence(
                new InstantCommand(d::resetPitch),
                new InstantCommand(d::resetHeading),
                new InstantCommand(() -> d.setPose(new Pose2d(1.75, 4.45, Rotation2d.fromDegrees(0))), d),
                new InstantCommand(() -> i.setSpeed(0.25)),
                Commands.deadline(Commands.waitSeconds(1), new SetArmSimpleCommand(e, ElevatorPosition.CUBE_TOP, IntakePosition.CUBE_TOP)),
                Commands.deadline(Commands.waitSeconds(0.125), new InstantCommand(() -> i.setSpeed(-0.25))),
                Commands.deadline(new InstantCommand(() -> i.setSpeed(0))),
                Commands.deadline(Commands.waitSeconds(1.5), new SetArmSimpleCommand(e, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND)),
                Commands.deadline(new InstantCommand(() -> i.setSpeed(0.5))),
                Commands.deadline(Commands.waitSeconds(2.85), new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/blue-score-shoot-balance-1"), new SetArmSimpleCommand(e, ElevatorPosition.BOTTOM, IntakePosition.CUBE_GROUND)),
                new InstantCommand(() -> i.setSpeed(0.25)),
                Commands.deadline(Commands.waitSeconds(3.9), new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/blue-score-shoot-balance-2"), new SetArmSimpleCommand(e, ElevatorPosition.CUBE_MID, IntakePosition.CUBE_MID)),
                Commands.deadline(Commands.waitSeconds(0.125), new InstantCommand(() -> i.setSpeed(-1))),
                new InstantCommand(() -> i.setSpeed(0)),
                Commands.deadline(Commands.waitSeconds(1.6), new YawLockedTranspose(d, new ChassisSpeeds(-0.75,0,0))),
                Commands.deadline(Commands.waitSeconds(5), new BalanceCommand(d, false))
        )),
         
        DO_NOTHING("Do Nothing", ((d,i,a) -> new InstantCommand(d::resetHeading))); // always put last

        Autos(String name, AutoChooser.CommandFactory commandFactory)
        {
            this.commandFactory = commandFactory;
            this.name = name;
        }

        Autos(CommandFactory commandFactory) {
            this.commandFactory = commandFactory;
            this.name = name().replace('_', ' ').toLowerCase();
        }
        public final CommandFactory commandFactory;
        public final String name;
}
