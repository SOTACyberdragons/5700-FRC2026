// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    public TalonFX hopperMotor = new TalonFX(Constants.IDs.HOPPER_MOTOR_ID);
	
    public DutyCycleOut hopperPercentOutput = new DutyCycleOut(0);
	private final CoastOut coastRequest = new CoastOut();

	/** Creates a new HopperSubsystem. */
	public HopperSubsystem() {
        setDefaultCommand(coastHopper());
	}

	public Command coastHopper() {
        return runOnce(() -> {
            hopperMotor.setControl(coastRequest);
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
