package com.team2502.robot2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

// 2022 import
public class TimeLeftCommand extends CommandBase { 
	private double time;

	/**
	* Wait until the specified time is remaining in the current match period, teleop or auto
	* @param time time (seconds) to wait until
	 */
	public TimeLeftCommand(double time) {
		this.time = time;
	}

	@Override
	public boolean isFinished() {
		return Timer.getMatchTime() < time;
	}
}
