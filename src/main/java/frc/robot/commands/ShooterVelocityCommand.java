// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterVelocityCommand extends Command {
	private ShooterSubsystem m_shooterSubsystem;
	private double velocity;

	private boolean isKilled = false;

    public ShooterVelocityCommand(ShooterSubsystem shooter, double velocity) {
		m_shooterSubsystem = shooter;
		addRequirements(m_shooterSubsystem);
		this.velocity = velocity;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putBoolean("Shooting", true);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_shooterSubsystem.runShooterVelocity(velocity);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		SmartDashboard.putBoolean("Shooting", false);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return isKilled;
	}
}
