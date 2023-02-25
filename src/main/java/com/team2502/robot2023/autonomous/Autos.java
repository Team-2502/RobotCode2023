package com.team2502.robot2023.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;

import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Elevator.ElevatorPosition;
import com.team2502.robot2023.Constants.Subsystems.Manipulator.ManipulatorPosition;
import com.team2502.robot2023.autonomous.AutoChooser.CommandFactory;
import com.team2502.robot2023.commands.FollowPathAbsoluteCommand;

/**
 * class for autonomous command groups
 * put new groups before the do nothing group
 * */
public enum Autos { // first auto is default
        SCORE_POINTS((d,i,e,m) -> Commands.sequence(
            Commands.deadline(Commands.waitSeconds(.5), new InstantCommand(() -> m.setSpeed(0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(2), new InstantCommand(() -> e.setLinearSpeed(-0.3))),
            new InstantCommand(() -> e.setLinearSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(4), new InstantCommand(() -> e.setPitchSpeed(-0.6))),
            new InstantCommand(() -> e.setPitchSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(.5), new InstantCommand(() -> m.setSpeed(-0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(4), new InstantCommand(() -> e.setPitchSpeed(0.6))),
            new InstantCommand(() -> e.setPitchSpeed(0.0)),
            Commands.deadline(Commands.waitSeconds(2), new InstantCommand(() -> e.setLinearSpeed(0.3))),
            new InstantCommand(() -> e.setLinearSpeed(0.0)),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
        )),

        SCORE_POINTS_PID((d,i,e,m) -> Commands.sequence(
            //new InstantCommand(() -> {
            //    m.home();
            //    m.set(ManipulatorPosition.CONE);
            //}),
            //Commands.waitSeconds(2),
            Commands.deadline(Commands.waitSeconds(.5), new InstantCommand(() -> m.setSpeed(0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            new InstantCommand(() -> e.set(ElevatorPosition.TOP)),
            Commands.waitSeconds(2),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.OUT)),
            Commands.waitSeconds(5),
            Commands.deadline(Commands.waitSeconds(.5), new InstantCommand(() -> m.setSpeed(-0.3))),
            new InstantCommand(() -> m.setSpeed(0.0)),
            //new InstantCommand(() -> m.set(ManipulatorPosition.OPEN)),
            //Commands.waitSeconds(.25),
            new InstantCommand(() -> e.setPitch(ElevatorPitch.STOWED)),
            Commands.waitSeconds(5),
            new InstantCommand(() -> e.set(ElevatorPosition.BOTTOM)),
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
        )),

        TEST_PATH((d,i,e,m) -> Commands.sequence(
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
