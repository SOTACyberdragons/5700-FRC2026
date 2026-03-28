// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPercentSetpoint;
import frc.robot.Constants.ShooterVelocitySetpoint;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartShootVelocityCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;
	private FeederSubsystem m_feederSubsystem;
	private ShooterVelocitySetpoint setpoint;
    /** Creates a new SmartShootPercentCommand. */
    public SmartShootVelocityCommand(ShooterSubsystem shooter, FeederSubsystem feeder, ShooterVelocitySetpoint setpoint) {
    	// Use addRequirements() here to declare subsystem dependencies.
		m_shooterSubsystem = shooter;
		m_feederSubsystem = feeder;
		this.setpoint = setpoint;
		addRequirements(m_shooterSubsystem);
		addRequirements(m_feederSubsystem);
		// math.abs(setpoint - get) < constant
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
		SmartDashboard.putBoolean("Shooting", true);
	}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
		m_shooterSubsystem.runShooterVelocity(setpoint.velocity);
		// if the current velocity is close enough (within the threshold) of the setpoint, run the kicker. this ensures not only that the kicker only runs when the shooter is up to speed, but also that if you decrease the speed, the shooter only shoots when at that speed
		if (m_shooterSubsystem.getLeaderMotorVelocitySignal() > (setpoint.velocity - Constants.FeederConstants.CLOSE_ENOUGH)) {
			// run kicker if shooter is at the right speed
			m_feederSubsystem.runFeederPercent(Constants.FeederPercentSetpoint.Feed.percent);
    	} else {
			m_feederSubsystem.runFeederPercent(0);
		}
		// run the shooter only if the speed is less than the setpoint
		// run only the shooter if it isn't up to speed yet
	}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
		SmartDashboard.putBoolean("Shooting", false);
	}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
