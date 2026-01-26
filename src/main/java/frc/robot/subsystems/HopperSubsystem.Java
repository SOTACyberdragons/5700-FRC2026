// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    private TalonFX hopperMotor = new TalonFX(Constants.IDs.HOPPER_MOTOR_ID);
    private DutyCycleOut percentOutput = new DutyCycleOut(0);
	/** Creates a new HopperSubsystem. */
	public HopperSubsystem() {

	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command runHopper() {
		return runOnce(() -> {
			
		});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
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
