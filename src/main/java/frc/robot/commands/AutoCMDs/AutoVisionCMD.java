// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class AutoVisionCMD extends Command {

	private CommandSwerveDrivetrain m_drivetrain;
	private VisionSubsystem m_visionSubsystem;

	private final PIDController rotationPID = new PIDController(
			Constants.VisionConstants.AUTO_ROTATION_PID_KP,
			Constants.VisionConstants.AUTO_ROTATION_PID_KI,
			Constants.VisionConstants.AUTO_ROTATION_PID_KD); // tx
	private final PIDController forwardPID = new PIDController(
			Constants.VisionConstants.AUTO_FORWARD_PID_KP,
			Constants.VisionConstants.AUTO_FORWARD_PID_KI,
			Constants.VisionConstants.AUTO_FORWARD_PID_KD); // ty
	private final PIDController lateralPID = new PIDController(
			Constants.VisionConstants.AUTO_LATERAL_PID_KP,
			Constants.VisionConstants.AUTO_LATERAL_PID_KI,
			Constants.VisionConstants.AUTO_LATERAL_PID_KD);

	private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.Velocity);

	private int direction;

	private double forwardCommand;
	private double lateralCommand;
	private double rotationCommand;
	private boolean killed;

	/** Creates a new VisionMoveToTarget. */
	public AutoVisionCMD(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionsubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.m_drivetrain = drivetrain;
		this.m_visionSubsystem = visionsubsystem;

		addRequirements(drivetrain, visionsubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("auto vision cmd");

		killed = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (m_visionSubsystem.getTargetVisibleLL1() || m_visionSubsystem.getTargetVisibleLL2()) {
			if (m_visionSubsystem.getTA2() >= m_visionSubsystem.getTA1()) {
				forwardCommand = forwardPID.calculate(m_visionSubsystem.getForward2(), Constants.VisionConstants.SETPOINT_FORWARD_COMMAND_TAG_2); // Target forward distance
				// (1 meter away) 0.477
				lateralCommand = lateralPID.calculate(m_visionSubsystem.getLateral2(), Constants.VisionConstants.SETPOINT_LATERAL_COMMAND_TAG_2);
				rotationCommand = rotationPID.calculate(m_visionSubsystem.getRotation2(), Constants.VisionConstants.SETPOINT_ROTATION_COMMAND_TAG_2);

			} else if (m_visionSubsystem.getTA2() < m_visionSubsystem.getTA1()) {
				forwardCommand = forwardPID.calculate(m_visionSubsystem.getForward(), Constants.VisionConstants.SETPOINT_FORWARD_COMMAND_TAG_1); // Target forward distance
				// (1 meter away) 0.477
				lateralCommand = lateralPID.calculate(m_visionSubsystem.getLateral(), Constants.VisionConstants.SETPOINT_LATERAL_COMMAND_TAG_1);
				rotationCommand = rotationPID.calculate(m_visionSubsystem.getRotation(), Constants.VisionConstants.SETPOINT_ROTATION_COMMAND_TAG_1);

			} else {
				forwardCommand = 0;// forwardSupplier.get();
				lateralCommand = 0;// lateralSupplier.get();
				rotationCommand = 0;// rotationSupplier.get();
			}
		} else {
			forwardCommand = 0;// forwardSupplier.get();
			lateralCommand = 0;// lateralSupplier.get();
			rotationCommand = 0;// rotationSupplier.get();
		}
		if (Math.abs(m_visionSubsystem.getRotation2()) < 0.1 || Math.abs(m_visionSubsystem.getRotation()) < 0.1) {
			rotationCommand = 0;
		}

		m_drivetrain.setControl(
			visionRequest.withVelocityX(forwardCommand * Constants.VisionConstants.AUTO_VISION_ADJUST_FORWARD)// forwardCommand
				.withVelocityY(lateralCommand * Constants.VisionConstants.AUTO_VISION_ADJUST_LATERAL)// lateralCommand (constant is negative)
				.withRotationalRate(rotationCommand * Constants.VisionConstants.AUTO_VISION_ADJUST_ROTATION)// rotationCommand
		);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return killed;
	}
}
