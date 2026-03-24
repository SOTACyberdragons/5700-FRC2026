// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPercentSetpoint;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterPercentDefaultCommand extends Command {
	private final ShooterSubsystem m_shooterSubsystem;

	private boolean isKilled = false;


	public ShooterPercentDefaultCommand(ShooterSubsystem shooterSubsystem) {
		m_shooterSubsystem = shooterSubsystem;

		addRequirements(m_shooterSubsystem);
	}


    // Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// set the motor to a basic speed for default
		m_shooterSubsystem.coastFlywheel();
		// m_shooterSubsystem.leaderMotorPercentOutput.withOutput(ShooterPercentSetpoint.Sleep.percent);
		// m_shooterSubsystem.leaderMotor.setControl(m_shooterSubsystem.leaderMotorPercentOutput);
		// m_shooterSubsystem.followerMotor.setControl(new Follower(Constants.IDs.SHOOTER_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
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
