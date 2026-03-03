// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
	private ShooterSubsystem m_shooterSubsystem;
	private final ShooterSetpoint shooterSetpoint;

	private boolean isKilled = false;

	public enum ShooterSetpoint {
        Outtake(Constants.ShooterConstants.OUTTAKE_PERCENT),
        Near(Constants.ShooterConstants.SNOOT_NEAR_PERCENT),
        Far(Constants.ShooterConstants.SHOOT_FAR_PERCENT);

        /** The velocity target of the setpoint. */
        public final double leaderMotorTargetPercentOutput;

        private ShooterSetpoint(double leaderMotorTargetPercentOutput) {
            this.leaderMotorTargetPercentOutput = leaderMotorTargetPercentOutput;
        }
    }

	/** Creates a new ShootCommand. */
	public ShootCommand(ShooterSubsystem shooter, ShooterSetpoint setpoint) {
		m_shooterSubsystem = shooter;
		shooterSetpoint = setpoint;
		addRequirements(m_shooterSubsystem);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// set the velocity of the velocityvoltage object to the setpoint of the command
		m_shooterSubsystem.leaderMotorPercentOutput.Output = shooterSetpoint.leaderMotorTargetPercentOutput;
		// control the motor with the velocityvoltage
        m_shooterSubsystem.leaderMotor.setControl(m_shooterSubsystem.leaderMotorPercentOutput);
		// set the follower motor to follow the leader motor
		m_shooterSubsystem.followerMotor.setControl(new Follower(Constants.IDs.SHOOTER_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
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
