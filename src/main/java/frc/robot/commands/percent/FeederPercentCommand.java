// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.percent;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederPercentCommand extends Command {
	private HopperSubsystem m_hopperSubsystem;
	private double feederDelay;
	private boolean runKicker;

	private boolean isKilled = false;

	public FeederPercentCommand(HopperSubsystem hopper, boolean runKicker) {
		m_hopperSubsystem = hopper;
		this.runKicker = runKicker;
		addRequirements(m_hopperSubsystem);
	}

	public FeederPercentCommand(HopperSubsystem hopper) {
		m_hopperSubsystem = hopper;
		this.runKicker = true;
		addRequirements(m_hopperSubsystem);
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
		if(Timer.getFPGATimestamp()>feederDelay){
			// run the kicker only if commanded to
			if (runKicker){
				m_hopperSubsystem.runFeederPercent();
			} else {
				m_hopperSubsystem.runFeederPercentWithoutKicker();
			}
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
