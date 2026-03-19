// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.velocity;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeVelocitySetpoint;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeVelocityCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	private boolean isKilled = false;


	public OuttakeVelocityCommand(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(intakeSubsystem);
	}


    // Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// set the controller to the correct value
		m_intakeSubsystem.intakVelocityVoltage.withVelocity(IntakeVelocitySetpoint.Outtake.velocity);
		// control the motor using the controller
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
