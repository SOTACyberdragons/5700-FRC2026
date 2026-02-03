// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.States.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePivotSetpoint;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakePivotToggleCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public IntakePivotToggleCommand(IntakeSubsystem subsystem) {
		m_intakeSubsystem = subsystem;
		// intakeAngle = angle;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Update global state to the opposite of the current setpoint
		frc.robot.States.intakeState = (m_intakeSubsystem.pivotSetpoint == IntakePivotSetpoint.Up) ? IntakeState.UP : IntakeState.DOWN;
		// Update setpoint to the opposite of the current setpoint
		m_intakeSubsystem.pivotSetpoint = (m_intakeSubsystem.pivotSetpoint == IntakePivotSetpoint.Up) ? IntakePivotSetpoint.Down :IntakePivotSetpoint.Up;
		// Command the pivot motor to move to the new setpoint
		m_intakeSubsystem.intakePivotPosition.withPosition(m_intakeSubsystem.pivotSetpoint.IntakePivotTarget);
		m_intakeSubsystem.intakePivotMotor.setControl(m_intakeSubsystem.intakePivotPosition);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
