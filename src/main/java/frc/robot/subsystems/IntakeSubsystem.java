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




    // private final TalonFXConfiguration PivotMotorConfigs = motorTalonFXInitialConfigs.clone()
    //         .withMotorOutput(
    //             motorTalonFXInitialConfigs.MotorOutput.clone()
    //                 .withInverted(InvertedValue.CounterClockwise_Positive)
    //         )
    //         .withFeedback(
    //             motorTalonFXInitialConfigs.Feedback.clone()
    //                 .withSensorToMechanismRatio(1)
    //         )
    //         .withSlot0(
    //             motorTalonFXInitialConfigs.Slot0.clone()
    //                 .withKP(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_CONFIG_KP)
    //                 .withKI(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_CONFIG_KI)
    //                 .withKD(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_CONFIG_KD)
    //                 .withKS(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_CONFIG_KS)
    //                 .withKV(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_CONFIG_KV)
    //                 .withKA(Constants.IntakeConstants.INTAKE_PIVOT_MOTOR_CONFIG_KA)
    //         );


	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

	}

	/**
	//  * Example command factory method.
	//  *
	//  * @return a command
	//  */
	// public Command exampleMethodCommand() {
	// 	// Inline construction of command goes here.
	// 	// Subsystem::RunOnce implicitly requires `this` subsystem.
	// 	return runOnce(
	// 		() -> {
	// 			/* one-time action goes here */
	// 		});
	// }

    /**
     * Drives the flywheel to the provided velocity setpoint.
     *
     * @param setpoint Function returning the setpoint to apply
     * @return Command to run
     */
    public Command setTarget(double target) {
        return run(() -> {
            intakVelocityVoltage.withVelocity(target);
            intakeMotor.setControl(intakVelocityVoltage);
        });
    }

    public double getPivotStateAsDouble() {
        // Return the current intake pivot setpoint based on the global state
        return frc.robot.States.intakeState == IntakeState.UP ? IntakeConstants.INTAKE_PIVOT_UP : IntakeConstants.INTAKE_PIVOT_DOWN;
    }

    public Command setPivot(double target) {
		return run(() -> {
            // Update global state so other code (RobotContainer, etc.) can observe pivot state
            frc.robot.States.intakeState = (target == IntakeConstants.INTAKE_PIVOT_UP) ? IntakeState.UP : IntakeState.DOWN;
            // Command the pivot motor to move to the requested position
            intakePivotPosition.withPosition(target);
            intakePivotMotor.setControl(intakePivotPosition);
		});
	}

    public Command togglePivot() {
        return run(() -> {
            double currentIntakePivotPos = getPivotStateAsDouble();
            setPivot(currentIntakePivotPos == IntakeConstants.INTAKE_PIVOT_UP ? IntakeConstants.INTAKE_PIVOT_DOWN : IntakeConstants.INTAKE_PIVOT_UP);
        });
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

    public Command coastPivotIntake() {
        return runOnce(() -> {
            intakePivotMotor.setControl(pivotCoastRequest);
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
