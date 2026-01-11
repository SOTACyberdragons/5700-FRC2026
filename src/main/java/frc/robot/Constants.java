// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int k_DRIVER_CONTROLLER_PORT = 0;
  }

  public static class DrivetrainConstants {
    public static final double MAX_ANGULAR_RATE = 0.75;  // 3/4 of a rotation per second max angular velocity
    public static final int SKEW_RATE_LIMITER_Y = 3;
    public static final int SKEW_RATE_LIMITER_X = 3;
    public static final int SKEW_RATE_LIMITER_ROTATION = 4;
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
}
