// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

	private final CANBus kCANBus = new CANBus("rio");

    public final TalonFX feederMotor = new TalonFX(Constants.IDs.FEEDER_MOTOR_ID, kCANBus);
    public final TalonFX kickerMotor = new TalonFX(Constants.IDs.SHOOTER_KICKER_MOTOR_ID, kCANBus);

    private final CoastOut coastRequest = new CoastOut();

    public final DutyCycleOut feederPercentOutput = new DutyCycleOut(0);
    public final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

	public final DutyCycleOut kickerMotorPercentOutput = new DutyCycleOut(0);
	public final VelocityVoltage kickerMotorVelocityVoltage = new VelocityVoltage(0);

	private final StatusSignal<AngularVelocity> kickerMotorVelocitySignal = kickerMotor.getVelocity(false);
    private final StatusSignal<Current> kickerMotorTorqueCurrentSignal = kickerMotor.getTorqueCurrent(false);
	

	/** Creates a new FeederSubsystem. */
	public FeederSubsystem() {
        setDefaultCommand(coastFeeder());
	}

	public Command coastFeeder() {
        return runOnce(() -> {
            feederMotor.setControl(coastRequest);
			kickerMotor.setControl(coastRequest);
        });
	}


	public AngularVelocity getKickerMotorVelocitySignal() {
        return kickerMotorVelocitySignal.getValue();
    }

    public Current getKickerMotorTorqueCurrentSignal() {
        return kickerMotorTorqueCurrentSignal.getValue();
    }

	public void runFeederPercent(double percent){
        kickerMotorPercentOutput.withOutput(Constants.FeederConstants.KICKER_PERCENT);
        kickerMotor.setControl(kickerMotorPercentOutput); 

		feederPercentOutput.withOutput(percent);
        feederMotor.setControl(feederPercentOutput);
    }

    public void runFeederVelocity(double velocity){
        kickerMotorVelocityVoltage.withVelocity(Constants.FeederConstants.KICKER_RPM);
        kickerMotor.setControl(kickerMotorVelocityVoltage);

        feederVelocityVoltage.withVelocity(velocity);
        feederMotor.setControl(feederVelocityVoltage);
    }

	public void runFeederPercentWithoutKicker(double percent){
		feederPercentOutput.withOutput(percent);
        feederMotor.setControl(feederPercentOutput);
    }

    public void runFeederVelocityWithoutKicker(double velocity){
        feederVelocityVoltage.withVelocity(velocity);
        feederMotor.setControl(feederVelocityVoltage);
    }

	public Trigger getTriggerWhenNearTargetVelocity(AngularVelocity threshold) {
        return new Trigger(() -> {
            boolean kicker = kickerMotorVelocitySignal.isNear(
                RotationsPerSecond.of(kickerMotorVelocityVoltage.Velocity), threshold
            );
            return kicker;
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
