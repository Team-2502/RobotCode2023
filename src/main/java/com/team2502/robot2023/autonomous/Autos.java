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

/**
 * class for autonomous command groups
 * put new groups before the do nothing group
 * */
public enum Autos { // first auto is default
        ENGAGE((d,i,e,m) -> Commands.sequence( 
            Commands.deadline(Commands.waitSeconds(1.65), new InstantCommand(() -> {d.setSpeeds(new ChassisSpeeds(-1,0,0));})),
            new BalanceCommand(d, false),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        ONE_CUBE_MID_ENGAGE_HETERO((d,i,e,m) -> Commands.sequence(
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
            Commands.deadline(Commands.waitSeconds(1.65), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
            new BalanceCommand(d, false),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        ONE_CUBE_SOUTH_BACKUP_HETERO((d,i,e,m) -> Commands.sequence(
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

        ONE_CUBE_SOUTH_BACKUP_TIMED((d,i,e,m) -> Commands.sequence(
            //new InstantCommand(() -> {
            //    m.home();
            //    m.set(ManipulatorPosition.CONE);
            //}),
            //Commands.waitSeconds(2),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(13.85,0.5259,Rotation2d.fromDegrees(180))), d),
            //Commands.deadline(Commands.waitSeconds(.75), new InstantCommand(() -> m.setSpeed(0.3))),
            //new InstantCommand(() -> m.setSpeed(0.0)),
            new InstantCommand(() -> e.set(ElevatorPosition.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            Commands.deadline(Commands.waitSeconds(.7), new InstantCommand(() -> {d.setSpeeds(new ChassisSpeeds(.8,0,0));})),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();}),
            Commands.deadline(Commands.waitSeconds(1.05), new InstantCommand(() -> m.setSpeed(-0.6))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.parallel(
                Commands.sequence(
                    Commands.deadline(Commands.waitSeconds(3.5), new InstantCommand(() -> {d.setSpeeds(new ChassisSpeeds(-.8,0,0));})),
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


        ONE_CUBE_SOUTH_BACKUP((d,i,e,m) -> Commands.sequence(
            //new InstantCommand(() -> {
            //    m.home();
            //    m.set(ManipulatorPosition.CONE);
            //}),
            //Commands.waitSeconds(2),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(13.85,0.5259,Rotation2d.fromDegrees(180))), d),
            //Commands.deadline(Commands.waitSeconds(.75), new InstantCommand(() -> m.setSpeed(0.3))),
            //new InstantCommand(() -> m.setSpeed(0.0)),
            new InstantCommand(() -> e.set(ElevatorPosition.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.CUBE_TOP)),
            Commands.waitSeconds(1.2),
            Commands.deadline(Commands.waitSeconds(2.8), new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/s1-1")),
            Commands.deadline(Commands.waitSeconds(1.25), new InstantCommand(() -> m.setSpeed(-0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.parallel(
                Commands.deadline(Commands.waitSeconds(5), new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/s1-2")),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
                    Commands.waitSeconds(2),
                    new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM))
                )
            )
        )),

        ONE_CONE_SOUTH_BACKUP((d,i,e,m) -> Commands.sequence(
            //new InstantCommand(() -> {
            //    m.home();
            //    m.set(ManipulatorPosition.CONE);
            //}),
            //Commands.waitSeconds(2),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(13.85,0.5259,Rotation2d.fromDegrees(180))), d),
            Commands.deadline(Commands.waitSeconds(.75), new InstantCommand(() -> m.setSpeed(0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            new InstantCommand(() -> e.set(ElevatorPosition.TOP)),
            Commands.waitSeconds(2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.OUT)),
            Commands.waitSeconds(2),
            new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/s1-1"),
            Commands.deadline(Commands.waitSeconds(.95), new InstantCommand(() -> m.setSpeed(-0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            //new InstantCommand(() -> m.set(ManipulatorPosition.OPEN)),
            //Commands.waitSeconds(2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
            Commands.waitSeconds(3),
            new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM)),
            new FollowPathAbsoluteCommand(d, "../pathplanner/generatedJSON/s1-2")
        )),

        ONE_CONE_SOUTH_NO_ENCODER((d,i,e,m) -> Commands.sequence(
            Commands.deadline(Commands.waitSeconds(.5), new InstantCommand(() -> m.setSpeed(0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(2), new InstantCommand(() -> e.setLinearSpeed(-0.3))),
            new InstantCommand(() -> e.setLinearSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(4), new InstantCommand(() -> e.setPitchSpeed(-0.2))),
            new InstantCommand(() -> e.setPitchSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(.5), new InstantCommand(() -> m.setSpeed(-0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(4), new InstantCommand(() -> e.setPitchSpeed(0.2))),
            new InstantCommand(() -> e.setPitchSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(2), new InstantCommand(() -> e.setLinearSpeed(0.3))),
            new InstantCommand(() -> e.setLinearSpeed(0.0)),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
        )),

        ONE_CODE_SOUTH_STATIC((d,i,e,m) -> Commands.sequence(
            //new InstantCommand(() -> {
            //    m.home();
            //    m.set(ManipulatorPosition.CONE);
            //}),
            //Commands.waitSeconds(2),
            Commands.deadline(Commands.waitSeconds(.75), new InstantCommand(() -> m.setSpeed(0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            new InstantCommand(() -> e.set(ElevatorPosition.TOP)),
            Commands.waitSeconds(2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.OUT)),
            Commands.waitSeconds(1.5),
            Commands.deadline(Commands.waitSeconds(.95), new InstantCommand(() -> m.setSpeed(-0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            //new InstantCommand(() -> m.set(ManipulatorPosition.OPEN)),
            //Commands.waitSeconds(2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
            Commands.waitSeconds(3),
            new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM)),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
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
