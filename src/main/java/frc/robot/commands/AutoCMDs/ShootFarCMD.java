package frc.robot.commands.AutoCMDs;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Example autonomous command that does nothing.
 * This is a template for creating new commands.
 */
public class ShootFarCMD extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    /**
     * Creates a new ExampleCMD
     */

    private boolean killed = false;

    public ShootFarCMD(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Add initialization code here
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        // Add execution code here
        m_shooterSubsystem.leaderMotorPercentOutput.withOutput(Constants.ShooterConstants.SHOOT_FAR_PERCENT);
		m_shooterSubsystem.leaderMotor.setControl(m_shooterSubsystem.leaderMotorVelocityVoltage);
        m_shooterSubsystem.followerMotor.setControl(new Follower(Constants.IDs.SHOOTER_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
        m_shooterSubsystem.kickerMotorPercentOutput.withOutput(Constants.ShooterConstants.KICKER_PERCENT);
        Commands.waitUntil(m_shooterSubsystem.getTriggerWhenNearTargetVelocity(RotationsPerSecond.of(Constants.ShooterConstants.SPINUP_THRESHOLD)));
        m_shooterSubsystem.kickerMotor.setControl(m_shooterSubsystem.kickerMotorVelocityVoltage);
        
    }

    /**
     * Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // Add cleanup code here
    }

    /**
     * Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return killed;
    }
}

