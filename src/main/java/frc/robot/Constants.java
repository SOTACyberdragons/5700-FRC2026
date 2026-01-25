// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class IDs {
    // intake
    public static final int INTAKE_MOTOR_ID = 0;
    
    public static final int INTAKE_PIVOT_MOTOR_ID = 0;
    public static final int INTAKE_TOP_INDEX_ID = 21;

    public static final int HOPPER_MOTOR_ID = 0;
    public static final int SHOOTER_LEADER_MOTOR_ID = 0;
    public static final int CLIMBER_MOTOR_ID = 0;
  }

  public static class DrivetrainConstants {
    public static final double MAX_ANGULAR_RATE = 0.75; // 3/4 of a rotation per second max angular velocity
    public static final int SKEW_RATE_LIMITER_Y = 3;
    public static final int SKEW_RATE_LIMITER_X = 3;
    public static final int SKEW_RATE_LIMITER_ROTATION = 4;
    public static final double ROBOT_CENTRIC_DRIVE_SPEED = 0.5;

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

      public static final double TOP_INDEX_MOTOR_CONFIG_KP = 0.8;
      public static final double TOP_INDEX_MOTOR_CONFIG_KI = 0;
      public static final double TOP_INDEX_MOTOR_CONFIG_KD = 0;
      public static final double TOP_INDEX_MOTOR_CONFIG_KS = 0;
      public static final double TOP_INDEX_MOTOR_CONFIG_KV = 0.12;
      public static final double TOP_INDEX_MOTOR_CONFIG_KA = 0;

      public static final double INTAKE_PIVOT_MOTOR_CONFIG_KP = 0.8;
      public static final double INTAKE_PIVOT_MOTOR_CONFIG_KI = 0;
      public static final double INTAKE_PIVOT_MOTOR_CONFIG_KD = 0;
      public static final double INTAKE_PIVOT_MOTOR_CONFIG_KS = 0;
      public static final double INTAKE_PIVOT_MOTOR_CONFIG_KV = 0.12;
      public static final double INTAKE_PIVOT_MOTOR_CONFIG_KA = 0;

      public static final double INTAKE_PIVOT_UP = 0;
      public static final double INTAKE_PIVOT_DOWN = 0;
  }

  public static class HopperConstants {

  }

  public static class ShooterConstants {
    public static final double SPINUP_THRESHOLD = 3.0;

    public static final double INTAKE_PRM = 80;
    public static final double OUTTAKE_RPM = -80;
    public static final double SHOOT_NEAR_RPM = 70;
    public static final double SHOOT_FAR_RPM = 80;

    public static final double LEADER_MOTOR_CONFIG_KP = 0.8;
    public static final double LEADER_MOTOR_CONFIG_KI = 0;
    public static final double LEADER_MOTOR_CONFIG_KD = 0;
    public static final double LEADER_MOTOR_CONFIG_KS = 0;
    public static final double LEADER_MOTOR_CONFIG_KV = 0.12;
    public static final double LEADER_MOTOR_CONFIG_KA = 0;

  }

  public static class ClimberConstants {

  }
}
