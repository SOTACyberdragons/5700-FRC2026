// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FeederPercentSetpoint;
import frc.robot.subsystems.FeederSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederPercentCommand extends Command {
	private FeederSubsystem m_feederSubsystem;
	private double feederDelay;
	private boolean runKicker;
	private FeederPercentSetpoint setpoint;

	private boolean isKilled = false;

	public FeederPercentCommand(FeederSubsystem feeder, boolean runKicker, FeederPercentSetpoint setpoint) {
		m_feederSubsystem = feeder;
		this.runKicker = runKicker;
		this.setpoint = setpoint;
		addRequirements(m_feederSubsystem);
	}

	public FeederPercentCommand(FeederSubsystem feeder, FeederPercentSetpoint setpoint) {
		m_feederSubsystem = feeder;
		this.runKicker = true;
		this.setpoint = setpoint;
		addRequirements(m_feederSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// initialize the timer for the delay
		feederDelay = Timer.getFPGATimestamp()+ Constants.ShooterConstants.FEEDER_DELAY;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// run only if the time has passed
		// if(Timer.getFPGATimestamp()>feederDelay){
			// run the kicker only if commanded to
			if (runKicker){
				m_feederSubsystem.runFeederPercent(setpoint.percent);
			} else {
				m_feederSubsystem.runFeederPercentWithoutKicker(setpoint.percent);
			}
		
		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return isKilled;
	}
}
