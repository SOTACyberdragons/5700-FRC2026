// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakePercentSetpoint;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePercentToggleCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	private boolean intakeIntaking;

	private boolean isKilled = false;


	public IntakePercentToggleCommand(IntakeSubsystem intakeSubsystem, boolean intakeIntaking) {
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
			// set the controller to the correct value
			m_intakeSubsystem.intakePercentOutput.withOutput(IntakePercentSetpoint.Intake.percent);
			// control the motor using the controller
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
