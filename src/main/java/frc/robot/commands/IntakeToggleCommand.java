// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeToggleCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	private boolean intakeIntaking;

	private boolean isKilled = false;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param intakeSubsystem The subsystem used by this command.
	 * 
	 * @param setpoint The setpoint to go to. Should be of type IntakeSetpoint
	 */
	public IntakeToggleCommand(IntakeSubsystem intakeSubsystem, boolean intakeIntaking) {
		m_intakeSubsystem = intakeSubsystem;
		this.intakeIntaking = intakeIntaking;


		addRequirements(intakeSubsystem);
	}


    // Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (intakeIntaking){ // if the intake was intaking, turn it off
			m_intakeSubsystem.coastIntake();
			this.intakeIntaking = false;
		} else { // otherwise, turn it on to the setpoint
			m_intakeSubsystem.intakePercentOutput.withOutput(Constants.IntakeConstants.INTAKE_PERCENT);
			m_intakeSubsystem.intakeMotor.setControl(m_intakeSubsystem.intakePercentOutput);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// when the command is finished (the driver stops pressing the button) set intakeIntaking to true if we were intaking
		this.intakeIntaking = true;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return isKilled;
	}
}
