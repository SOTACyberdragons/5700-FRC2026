// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

	    /** Velocity setpoints for the flywheel. */
    public enum IntakeSetpoint {
        Intake(RotationsPerSecond.of(80)),
        Outtake(RotationsPerSecond.of(-70)),
        FeedToShoot(RotationsPerSecond.of(-90));

        /** The velocity target of the setpoint. */
        public final AngularVelocity TopIndexTarget;

        private IntakeSetpoint(AngularVelocity TopIndexTarget) {
            this.TopIndexTarget = TopIndexTarget;
        }
    }

	private final CANBus kCANBus = new CANBus("*");
	private final VelocityVoltage TopIndexSetpointRequest = new VelocityVoltage(0);
	private final CoastOut coastRequest = new CoastOut();
    private final TalonFX TopIndex = new TalonFX(21, kCANBus);


	private TalonFX intakePivotMotor = new TalonFX(Constants.IDs.INTAKE_PIVOT_MOTOR_ID);
	private TalonFX intakeMotor = new TalonFX(Constants.IDs.INTAKE_MOTOR_ID);

	// Control Modes for Intake Pivot
	private PositionDutyCycle intakePivotPosition = new PositionDutyCycle(0);
	private MotionMagicVoltage intakePivotMM = new MotionMagicVoltage(0);

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
			() -> {
				/* one-time action goes here */
			});
	}

	    /**
     * Drives the flywheel to the provided velocity setpoint.
     *
     * @param setpoint Function returning the setpoint to apply
     * @return Command to run
     */
    public Command setTarget(Supplier<IntakeSetpoint> target) {
        return run(() -> {
            IntakeSetpoint t = target.get();
            TopIndexSetpointRequest.withVelocity(t.TopIndexTarget);
            TopIndex.setControl(TopIndexSetpointRequest);
        });
    }

	    /**
     * Stops driving the Intake. We use coast so no energy is used during the braking event.
     *
     * @return Command to run
     */
    public Command coastIntake() {
        return runOnce(() -> {
            TopIndex.setControl(coastRequest);
        });
    }

	/**
	 * An example method querying a boolean state of the subsystem (for example, a
	 * digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
