// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
	private ShooterSubsystem m_shooterSubsystem;
	private final ShooterSetpoint shooterSetpoint;

	private boolean isKilled = false;

	public enum ShooterSetpoint {
        Feed(RotationsPerSecond.of(Constants.ShooterConstants.FEED_RPM)),
        Outtake(RotationsPerSecond.of(Constants.ShooterConstants.OUTTAKE_RPM)),
        Near(RotationsPerSecond.of(Constants.ShooterConstants.SHOOT_NEAR_RPM)),
        Far(RotationsPerSecond.of(Constants.ShooterConstants.SHOOT_FAR_RPM));

        /** The velocity target of the setpoint. */
        public final AngularVelocity leaderMotorTarget;

        private ShooterSetpoint(AngularVelocity leaderMotorTarget) {
            this.leaderMotorTarget = leaderMotorTarget;
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
		m_shooterSubsystem.leaderMotorSetpointRequest.withVelocity(shooterSetpoint.leaderMotorTarget);
        m_shooterSubsystem.leaderMotor.setControl(m_shooterSubsystem.leaderMotorSetpointRequest);
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
