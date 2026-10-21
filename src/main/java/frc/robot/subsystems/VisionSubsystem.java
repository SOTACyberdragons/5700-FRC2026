// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    // private final PIDController rotationPID = new PIDController(0.05,0.0001,0); //tx
    // private final PIDController forwardPID = new PIDController(2,0.05,0); //ty
    // private final PIDController lateralPIDR = new PIDController(2, 0.05, 0);
    // private final PIDController lateralPIDL = new PIDController(2, 0.05, 0);
    // private final PIDController lateralPID = new PIDController(2, 0.05, 0);
    // private static final double DESIRED_FORWARD = 1.0; // desired forward distance
    // private static final double DESIRED_LATERAL = 0.0; // desired lateral offset (centered)
    private double forwardCommand;
    private double lateralCommand;
    private double rotationCommand;

    private double[] targetPose = new double[6];

    private double currentForward;
    private double currentLateral;
    private double currentYaw;

    private boolean targetVisible = false;

    private double ta;

    /**
     * Creates a new Vision.
     */
    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        targetVisible = LimelightHelpers.getTV(Constants.IDs.LIMELIGHT_NAME); // Valid target flag

        if (targetVisible) {
            targetPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.IDs.LIMELIGHT_NAME);

            currentForward = targetPose[2];
            currentLateral = targetPose[0];
            currentYaw = targetPose[4];
        } else {
            currentForward = 0;
            currentLateral = 0;
            currentYaw = 0;
        }

        SmartDashboard.putNumber("Translation X", currentForward);
        SmartDashboard.putNumber("Translation Y", currentLateral);
        SmartDashboard.putNumber("ROT Z", currentYaw);

        ta = LimelightHelpers.getTA(Constants.IDs.LIMELIGHT_NAME);
        SmartDashboard.putNumber("ta", ta);

    }

    public boolean getTargetVisible() {
        return targetVisible = LimelightHelpers.getTV(Constants.IDs.LIMELIGHT_NAME);
    }

    public double getForwardCommand() {
        return forwardCommand;
    }

    public double getLateralCommand() {
        return lateralCommand;
    }

    public double getRotationCommand() {
        return rotationCommand;
    }

    public double getForward() {
        double ty = LimelightHelpers.getTY(Constants.IDs.LIMELIGHT_NAME);
        double trueForward = (Constants.VisionConstants.HUB_TAG_HEIGHT - Constants.VisionConstants.CAMERA_MOUNT_HEIGHT) / tan(toRadians(Constants.VisionConstants.CAMERA_MOUNT_ANGLE + ty));
        return trueForward;
    }

    public double getLateral() {
        return currentLateral;
    }

    public double getRotation() {
        return currentYaw;
    }

    public double getTA() {
        ta = LimelightHelpers.getTA(Constants.VisionConstants.APRIL_TAG_1_NAME);
        SmartDashboard.putNumber("ta", ta);
        return ta;
    }

}
