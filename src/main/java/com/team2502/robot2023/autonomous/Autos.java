package com.team2502.robot2023.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPitch;
import com.team2502.robot2023.Constants.Subsystems.Arm.ElevatorPosition;
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
        ENGAGE((d,i,a) -> Commands.sequence( 
            new InstantCommand(d::resetPitch),
            new InstantCommand(d::resetHeading),
            Commands.deadline(Commands.waitSeconds(1.65), new YawLockedTranspose(d, new ChassisSpeeds(-1,0,0))),
            Commands.deadline(new TimeLeftCommand(1), new BalanceCommand(d, false)),
            Commands.deadline(Commands.waitSeconds(.5), new YawLockedTranspose(d, new ChassisSpeeds(0,-.3,0))),
            new InstantCommand(() -> {d.setPowerNeutralMode(NeutralMode.Brake); d.stop();})
        )),

        LEAVE_COMMUNITY("Leave CT spawn", (d,i,e) -> Commands.sequence(
            new InstantCommand(d::resetHeading),
            new InstantCommand(() -> d.setPose(new Pose2d(14.693,4.678,Rotation2d.fromDegrees(180))), d),
            new FollowPathAbsoluteCommand(d, "testpath")
        )),
         
        DO_NOTHING("Do Nothing", ((d,i,e) -> new InstantCommand(d::resetHeading))); // always put last

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
