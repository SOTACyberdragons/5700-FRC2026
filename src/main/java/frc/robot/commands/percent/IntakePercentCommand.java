// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.percent;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakePercentSetpoint;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePercentCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	private boolean isKilled = false;

	public IntakePercentCommand(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(intakeSubsystem);
	}


    // Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// change the controller to the correct value
		m_intakeSubsystem.intakePercentOutput.withOutput(IntakePercentSetpoint.Intake.percent);

		// run the motor using the controller
		m_intakeSubsystem.intakeMotor.setControl(m_intakeSubsystem.intakePercentOutput);

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
