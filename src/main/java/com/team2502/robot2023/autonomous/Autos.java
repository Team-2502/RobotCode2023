package com.team2502.robot2023.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPosition;
import com.team2502.robot2023.autonomous.AutoChooser.CommandFactory;
import com.team2502.robot2023.commands.BalanceCommand;
import com.team2502.robot2023.commands.FollowPathAbsoluteCommand;
import com.team2502.robot2023.commands.YawLockedTranspose;
import com.team2502.robot2023.commands.TimeLeftCommand;

/**
 * class for autonomous command groups
 * put new groups before the do nothing group
 * */
public enum Autos { // first auto is default
        ONE_CUBE_MID_ENGAGE_HETERO((d,i,e,m) -> Commands.sequence(
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(13.85,0.5259,Rotation2d.fromDegrees(180))), d),
            new InstantCommand(() -> e.set(ElevatorPosition.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            Commands.deadline(Commands.waitSeconds(.57), new YawLockedTranspose(d, new ChassisSpeeds(.8,0,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();}),
            Commands.deadline(Commands.waitSeconds(1.05), new InstantCommand(() -> m.setSpeed(-0.6))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(0.57), new YawLockedTranspose(d, new ChassisSpeeds(-.8,0,0))),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM)),
            Commands.deadline(Commands.waitSeconds(1.55), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
            Commands.deadline(new TimeLeftCommand(0.75), new BalanceCommand(d, false)),
            Commands.deadline(Commands.waitSeconds(.25), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        ONE_CUBE_SOUTH_BACKUP_HETERO((d,i,e,m) -> Commands.sequence(
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(13.85,0.5259,Rotation2d.fromDegrees(180))), d),
            new InstantCommand(() -> e.set(ElevatorPosition.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            Commands.deadline(Commands.waitSeconds(.57), new YawLockedTranspose(d, new ChassisSpeeds(.8,0,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();}),
            Commands.deadline(Commands.waitSeconds(1.05), new InstantCommand(() -> m.setSpeed(-0.6))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.parallel(
                Commands.sequence(
                    Commands.deadline(Commands.waitSeconds(3.5), new YawLockedTranspose(d, new ChassisSpeeds(-.8,0,0))),
                    new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
                ),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
                    Commands.waitSeconds(2),
                    new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM))
                )
            )
        )),

        ONE_CUBE_MID_LEAVE_ENGAGE_HETERO((d,i,e,m) -> Commands.sequence( // score one cube mid, leave community and balance
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(13.85,0.5259,Rotation2d.fromDegrees(180))), d),
            new InstantCommand(() -> e.set(ElevatorPosition.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            Commands.deadline(Commands.waitSeconds(.57), new YawLockedTranspose(d, new ChassisSpeeds(.8,0,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();}),
            Commands.deadline(Commands.waitSeconds(1.05), new InstantCommand(() -> m.setSpeed(-0.6))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(0.57), new YawLockedTranspose(d, new ChassisSpeeds(-.8,0,0))),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM)),
            Commands.deadline(Commands.waitSeconds(1.55), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
            Commands.deadline(Commands.waitSeconds(1.5), new YawLockedTranspose(d, new ChassisSpeeds(-0.9,0,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();}),
            Commands.waitSeconds(0.2),
            Commands.deadline(Commands.waitSeconds(1.5), new YawLockedTranspose(d, new ChassisSpeeds(0.9,0,0))),
            Commands.deadline(new TimeLeftCommand(0.75), new BalanceCommand(d, false)),
            Commands.deadline(Commands.waitSeconds(.25), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        ENGAGE((d,i,e,m) -> Commands.sequence( 
            Commands.deadline(Commands.waitSeconds(1.65), new InstantCommand(() -> {d.setSpeeds(new ChassisSpeeds(-1,0,0));})),
            new BalanceCommand(d, false),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        ENGAGE_HETERO((d,i,e,m) -> Commands.sequence( 
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            Commands.deadline(Commands.waitSeconds(1.65), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
            Commands.deadline(new TimeLeftCommand(1), new BalanceCommand(d, false)),
            Commands.deadline(Commands.waitSeconds(.5), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        LEAVE_COMMUNITY("Leave CT spawn", (d,i,e,m) -> Commands.sequence(
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
        )),
         
        DO_NOTHING("Do Nothing", ((d,i,e,m) -> new InstantCommand(d::resetHeading))); // always put last

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
