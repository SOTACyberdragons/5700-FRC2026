// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.States.IntakeState;

public class IntakeSubsystem extends SubsystemBase {


	private final CANBus kCANBus = new CANBus("*");

	/* controls used by the leader motors */
	public final VelocityVoltage intakVelocityVoltage = new VelocityVoltage(0);
	private final CoastOut coastRequest = new CoastOut();

	// Control Modes for Intake Pivot
	public PositionDutyCycle intakePivotPosition = new PositionDutyCycle(0);
	private MotionMagicVoltage intakePivotMM = new MotionMagicVoltage(0);
	private final CoastOut pivotCoastRequest = new CoastOut();

	public TalonFX intakePivotMotor = new TalonFX(Constants.IDs.INTAKE_PIVOT_MOTOR_ID, kCANBus);
	public TalonFX intakeMotor = new TalonFX(Constants.IDs.INTAKE_MOTOR_ID, kCANBus);

    public double intakeAngle = 0;


    // setpoints for intake and intake pivot
    public enum IntakeSetpoint {
        Intake(RotationsPerSecond.of(Constants.IntakeConstants.INTAKE_RPM)),
        Outtake(RotationsPerSecond.of(Constants.IntakeConstants.OUTTAKE_RPM)),
        FeedToShoot(RotationsPerSecond.of(Constants.IntakeConstants.FEED_TO_SHOOT_RPM));
        /** The velocity target of the setpoint. */
        public final AngularVelocity TopIndexTarget;

        private IntakeSetpoint(AngularVelocity TopIndexTarget) {
            this.TopIndexTarget = TopIndexTarget;
        }
    }

    public enum IntakePivotSetpoint {
		Up(Constants.IntakeConstants.INTAKE_PIVOT_UP),
		Down(Constants.IntakeConstants.INTAKE_PIVOT_DOWN);
		
		public final double IntakePivotTarget; // should probably be a different type
		
		private IntakePivotSetpoint(double IntakePivotTarget) {
			this.IntakePivotTarget = IntakePivotTarget;
		}
    }

    public IntakePivotSetpoint pivotSetpoint;


	/* device status signals */
    private final StatusSignal<AngularVelocity> intakeMotorVelocity = intakeMotor.getVelocity(false);
    private final StatusSignal<Current> intakeMotorTorqueCurrent = intakeMotor.getTorqueCurrent(false);

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

	}

	/**
     * Stops driving the Intake. We use coast so no energy is used during the braking event.
     *
     * @return Command to run
     */
    public Command coastIntake() {
        return runOnce(() -> {
            intakeMotor.setControl(coastRequest);
        });
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
