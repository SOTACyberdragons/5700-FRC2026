
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OuttakeCMD extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	private boolean isKilled = false;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param intakeSubsystem The subsystem used by this command.
	 * 
	 * @param setpoint The setpoint to go to. Should be of type IntakeSetpoint
	 */
	public OuttakeCMD(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(intakeSubsystem);
	}


    // Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_intakeSubsystem.intakVelocityVoltage.withVelocity(Constants.IntakeConstants.OUTTAKE_RPM);
		m_intakeSubsystem.intakeMotor.setControl(m_intakeSubsystem.intakVelocityVoltage);

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
