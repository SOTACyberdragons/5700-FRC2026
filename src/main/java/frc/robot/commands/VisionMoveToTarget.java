// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class VisionMoveToTarget extends Command {

	private CommandSwerveDrivetrain m_drivetrain;
	private VisionSubsystem m_visionSubsystem;

	private final PIDController rotationPID = new PIDController(
		Constants.VisionConstants.ROTATION_PID_KP,
		Constants.VisionConstants.ROTATION_PID_KI,
		Constants.VisionConstants.ROTATION_PID_KD); // tx
	private final PIDController forwardPID = new PIDController(
		Constants.VisionConstants.FORWARD_PID_KP,
		Constants.VisionConstants.FORWARD_PID_KI,
		Constants.VisionConstants.FORWARD_PID_KD); // ty
	private final PIDController lateralPID = new PIDController(
		Constants.VisionConstants.LATERAL_PID_KP,
		Constants.VisionConstants.LATERAL_PID_KI,
		Constants.VisionConstants.LATERAL_PID_KD);

	private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private int direction;

	private double forwardCommand;
	private double lateralCommand;
	private double rotationCommand;

	private Supplier<Double> forwardSupplier;
	private Supplier<Double> lateralSupplier;
	private Supplier<Double> rotationSupplier;

	/** Creates a new VisionMoveToTarget. */
	public VisionMoveToTarget(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem,
		Supplier<Double> forwardSupplier,
		Supplier<Double> lateralSupplier,
		Supplier<Double> rotationSupplier
	) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.forwardSupplier = forwardSupplier;
		this.lateralSupplier = lateralSupplier;
		this.rotationSupplier = rotationSupplier;
		this.m_drivetrain = drivetrain;
		this.m_visionSubsystem = visionSubsystem;

		addRequirements(drivetrain, visionSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("vision cmd");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (m_visionSubsystem.getTargetVisibleLL1() || m_visionSubsystem.getTargetVisibleLL2()) {
			if (m_visionSubsystem.getTA2() >= m_visionSubsystem.getTA1()) { // tag 2 is closer than or equal to tag 1
				forwardCommand = forwardPID.calculate(m_visionSubsystem.getForward2(), Constants.VisionConstants.SETPOINT_FORWARD_COMMAND_TAG_2); // Target forward distance
				// (1 meter
				// away) 0.477
				lateralCommand = lateralPID.calculate(m_visionSubsystem.getLateral2(), Constants.VisionConstants.SETPOINT_LATERAL_COMMAND_TAG_2);
				rotationCommand = rotationPID.calculate(m_visionSubsystem.getRotation2(), Constants.VisionConstants.SETPOINT_ROTATION_COMMAND_TAG_2);
			} else if (m_visionSubsystem.getTA2() < m_visionSubsystem.getTA1()) { // tag 2 is further than tag 1
				forwardCommand = forwardPID.calculate(m_visionSubsystem.getForward(), Constants.VisionConstants.SETPOINT_FORWARD_COMMAND_TAG_1); // Target forward distance (1
				// meter
				// away) 0.477
				lateralCommand = lateralPID.calculate(m_visionSubsystem.getLateral(), Constants.VisionConstants.SETPOINT_LATERAL_COMMAND_TAG_1);
				rotationCommand = rotationPID.calculate(m_visionSubsystem.getRotation(), Constants.VisionConstants.SETPOINT_ROTATION_COMMAND_TAG_1);
			} else { // this should never run
				forwardCommand = 0;// forwardSupplier.get();
				lateralCommand = 0;// lateralSupplier.get();
				rotationCommand = 0;// rotationSupplier.get();
			}
		} else { // cannot see either tag
			forwardCommand = 0;// forwardSupplier.get();
			lateralCommand = 0;// lateralSupplier.get();
			rotationCommand = 0;// rotationSupplier.get();
		}
		if (Math.abs(m_visionSubsystem.getRotation2()) < 0.5 || Math.abs(m_visionSubsystem.getRotation()) < 0.5) {
			rotationCommand = 0;
		}// what is this ???
		m_drivetrain.setControl(
			visionRequest.withVelocityX(forwardCommand)// forwardCommand
				.withVelocityY(-lateralCommand)// -lateralCommand
				.withRotationalRate(rotationCommand * 0.5)// rotationCommand
		);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// // Stop them_drivetrain when the command ends
		//m_drivetrain.setControl(
		// visionRequest.withVelocityX(0)
		// .withVelocityY(0)
		// .withRotationalRate(0)
		// );
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
