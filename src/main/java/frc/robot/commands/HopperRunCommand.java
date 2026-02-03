// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperRunCommand extends Command {
	private final HopperSubsystem m_hopperSubsystem;
    private boolean isKilled = false;

	/** Creates a new HopperCommand. */
	public HopperRunCommand(HopperSubsystem hopper) {
		m_hopperSubsystem = hopper;
		addRequirements(m_hopperSubsystem);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        m_hopperSubsystem.hopperPercentOutput.withOutput(Constants.HopperConstants.FEED_TO_SHOOT_RPM);
        m_hopperSubsystem.hopperMotor.setControl(m_hopperSubsystem.hopperPercentOutput);
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
