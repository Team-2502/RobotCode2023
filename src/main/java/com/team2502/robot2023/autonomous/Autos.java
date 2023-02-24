package com.team2502.robot2023.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import com.team2502.robot2023.autonomous.AutoChooser.CommandFactory;

/**
 * class for autonomous command groups
 * put new groups before the do nothing group
 * */
public enum Autos { // first auto is default
        DO_NOTHING("Do Nothing", ((d,i) -> new InstantCommand(d::resetHeading))); // always put last

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
