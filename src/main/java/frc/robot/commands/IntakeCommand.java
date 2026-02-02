// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSetpoint;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	private final IntakeSetpoint intakeSetpoint;
	private boolean isKilled = false;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 * 
	 * @param setpoint The setpoint to go to. Should be of type IntakeSetpoint
	 */
	public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeSetpoint setpoint) {
		m_intakeSubsystem = intakeSubsystem;
		intakeSetpoint = setpoint;

		addRequirements(intakeSubsystem);
	}


    // Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_intakeSubsystem.intakVelocityVoltage.withVelocity(intakeSetpoint.TopIndexTarget);
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
