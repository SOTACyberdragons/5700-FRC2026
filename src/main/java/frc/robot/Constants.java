// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int k_DRIVER_CONTROLLER_PORT = 0;

		/** Configs common across all motors. */
		static final TalonFXConfiguration motorTalonFXInitialConfigs = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Brake)
			)
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(Amps.of(120))
					.withStatorCurrentLimitEnable(true)
			);
	}

	public static class IDs {
		// intake
		public static final int INTAKE_MOTOR_ID = 19;

		public static final int FEEDER_MOTOR_ID = 18;

		public static final int SHOOTER_LEADER_MOTOR_ID = 14;
		public static final int SHOOTER_FOLLOWER_MOTOR_ID = 17;
		public static final int SHOOTER_KICKER_MOTOR_ID = 16;
	}

	public static class DrivetrainConstants {
		public static final double MAX_ANGULAR_RATE = 0.75; // 3/4 of a rotation per second max angular velocity
		public static final double MAX_SPEED_MULTIPLIER = 0.6;
		public static final int SKEW_RATE_LIMITER_Y = 3;
		public static final int SKEW_RATE_LIMITER_X = 3;
		public static final int SKEW_RATE_LIMITER_ROTATION = 4;
		public static final double ROBOT_CENTRIC_DRIVE_SPEED = 0.5;

		// hub-centric drive control
		public static final double K_P_HUB_CENTRIC = 10.0;
		public static final double K_I_HUB_CENTRIC = 0.0;
		public static final double K_D_HUB_CENTRIC = 0.0;


	}

	public static class VisionConstants {
		// VisionMoveToTarget.java ln 24
		public static final double ROTATION_PID_KP = 0.5;
		public static final double ROTATION_PID_KI = 0;
		public static final double ROTATION_PID_KD = 0;

		public static final double FORWARD_PID_KP = 2.5;
		public static final double FORWARD_PID_KI = 0;
		public static final double FORWARD_PID_KD = 0;

		public static final double LATERAL_PID_KP = 2;
		public static final double LATERAL_PID_KI = 0;
		public static final double LATERAL_PID_KD = 0.01;

		// AutoVisionCMD.java ln 23
		public static final double AUTO_ROTATION_PID_KP = 0.05;
		public static final double AUTO_ROTATION_PID_KI = 0;
		public static final double AUTO_ROTATION_PID_KD = 0;

		public static final double AUTO_FORWARD_PID_KP = 2.1;
		public static final double AUTO_FORWARD_PID_KI = 0;
		public static final double AUTO_FORWARD_PID_KD = 0;

		public static final double AUTO_LATERAL_PID_KP = 2.3;
		public static final double AUTO_LATERAL_PID_KI = 0;
		public static final double AUTO_LATERAL_PID_KD = 0.01;

		// VisionSubsystem.java ln 222
		public static final String APRIL_TAG_1_NAME = "limelight-left";
		public static final String APRIL_TAG_2_NAME = "limelight-right";

		// VisionMoveToTarget.java ln 76, AutoVisionCMD ln 66
		// in order of use (yes, tag 2 comes first)
		public static final double SETPOINT_FORWARD_COMMAND_TAG_2 = 0.32;
		public static final double SETPOINT_LATERAL_COMMAND_TAG_2 = -0.28;
		public static final double SETPOINT_ROTATION_COMMAND_TAG_2 = 0;

		public static final double SETPOINT_FORWARD_COMMAND_TAG_1 = 0.32;
		public static final double SETPOINT_LATERAL_COMMAND_TAG_1 = -0.02;
		public static final double SETPOINT_ROTATION_COMMAND_TAG_1 = 0;

		// AutoVisionCMD ln 93
		public static final double AUTO_VISION_ADJUST_FORWARD = 0.85;
		public static final double AUTO_VISION_ADJUST_LATERAL = -0.85;
		public static final double AUTO_VISION_ADJUST_ROTATION = 0.5;
	}

	public static class IntakeConstants {
		public static final double INTAKE_RPM = 80;
		public static final double OUTTAKE_RPM = -70;
		public static final double FEED_TO_SHOOT_RPM = 90; // -90

		public static final double INTAKE_PERCENT = -0.35;
		public static final double OUTTAKE_PERCENT = 0.5;
		public static final double SLEEP_PERCENT = 0.00;
	}

    public enum IntakePercentSetpoint {
        Outtake(IntakeConstants.OUTTAKE_PERCENT),
        Intake(IntakeConstants.INTAKE_PERCENT),
        Sleep(IntakeConstants.SLEEP_PERCENT);

        /** The velocity target of the setpoint. */
        public final double percent;

        private IntakePercentSetpoint(double percent) {
            this.percent = percent;
        }
    }

    public enum IntakeVelocitySetpoint {
        Outtake(Constants.IntakeConstants.OUTTAKE_PERCENT),
        Intake(Constants.IntakeConstants.INTAKE_PERCENT),
        Sleep(Constants.IntakeConstants.SLEEP_PERCENT);

        /** The velocity target of the setpoint. */
        public final double velocity;

        private IntakeVelocitySetpoint(double velocity) {
            this.velocity = velocity;
        }
    }

	public static class FeederConstants {
        public static final double KICKER_PERCENT = 0.6;
        public static final double KICKER_RPM = 90.0;

		public static final double CLOSE_ENOUGH = 1;

        public static final double INTAKE_RPM = 90; // -90
		public static final double FEED_RPM = 100;
		public static final double OUTTAKE_RPM = -90;
		public static final double SLEEP_RPM = 0.0;

		public static final double INTAKE_PERCENT = 0.4;
		public static final double FEED_PERCENT = 0.5;
		public static final double OUTTAKE_PERCENT = -0.4;
		public static final double SLEEP_PERCENT = 0.0;

	}		

	public enum FeederPercentSetpoint {
		Intake(Constants.FeederConstants.INTAKE_PERCENT),
		Feed(Constants.FeederConstants.FEED_PERCENT),
		Outtake(Constants.FeederConstants.OUTTAKE_PERCENT),
		Sleep(Constants.FeederConstants.SLEEP_PERCENT);


		/** The velocity target of the setpoint. */
		public final double percent;

		private FeederPercentSetpoint(double percent) {
			this.percent = percent;
		}
	}

	public enum FeederVelocitySetpoint {
		Intake(Constants.FeederConstants.INTAKE_RPM),
		Feed(Constants.FeederConstants.FEED_RPM),
		Outtake(Constants.FeederConstants.OUTTAKE_RPM),
		Sleep(Constants.FeederConstants.SLEEP_RPM);


		/** The velocity target of the setpoint. */
		public final double velocity;

		private FeederVelocitySetpoint(double velocity) {
			this.velocity = velocity;
		}
	}

	public static class ShooterConstants {
		public static final double FEEDER_DELAY = 2.2;
		public static final double FEEDER_SPINDOWN_DELAY = 1.0;

		public static final double OUTTAKE_PERCENT = 0.5;
		public static final double SHOOT_FAR_PERCENT = 1.0;
		public static final double SNOOT_NEAR_PERCENT = 0.8;
		public static final double SLEEP_PERCENT = 0.05;

		public static final double LEADER_MOTOR_CONFIG_KP = 1.5;
		public static final double LEADER_MOTOR_CONFIG_KI = 0.1;
		public static final double LEADER_MOTOR_CONFIG_KD = 0;
		public static final double LEADER_MOTOR_CONFIG_KS = 0.11;
		public static final double LEADER_MOTOR_CONFIG_KV = 0.12;
		public static final double LEADER_MOTOR_CONFIG_KA = 0;

		public static final double OUTTAKE_RPM = 0.0;
		public static final double SNOOT_NEAR_RPM = 40.0;
		public static final double SHOOT_FAR_RPM = 65.0;
		public static final double SLEEP_RPM = 0.0;
	}

	public static class LEDConstants {
		public static final int PWM_PORT = 1;
		public static final int LED_LENGTH = 38;
		public static final double BLINK_TIME = 0.1;
		
		public static final Color COLOR_GREEN = new Color(0,255,0);
		public static final Color COLOR_RED = new Color(255,0,0);
		public static final Color COLOR_BLUE = new Color(0,0,255);
		public static final Color COLOR_YELLOW = new Color(255,255,0);
		public static final Color COLOR_PURPLE = new Color(255,0,255);
		public static final Color COLOR_ORANGE = new Color(255,127,0);
	}

	
	public enum ShooterPercentSetpoint {
		Outtake(Constants.ShooterConstants.OUTTAKE_PERCENT),
		Near(Constants.ShooterConstants.SNOOT_NEAR_PERCENT),
		Far(Constants.ShooterConstants.SHOOT_FAR_PERCENT),
		Sleep(Constants.ShooterConstants.SLEEP_PERCENT);


		/** The velocity target of the setpoint. */
		public final double percent;

		private ShooterPercentSetpoint(double percent) {
			this.percent = percent;
		}
	}

	public enum ShooterVelocitySetpoint {
		Outtake(Constants.ShooterConstants.OUTTAKE_RPM),
		Near(Constants.ShooterConstants.SNOOT_NEAR_RPM),
		Far(Constants.ShooterConstants.SHOOT_FAR_RPM),
		Sleep(Constants.ShooterConstants.SLEEP_RPM);

		/** The velocity target of the setpoint. */
		public final double velocity;

		private ShooterVelocitySetpoint(double velocity) {
			this.velocity = velocity;
		}
	}
}