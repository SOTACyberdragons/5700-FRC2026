// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
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

    private double[] tagPose = new double[6];

    private double currentTagDistForward;
    private double currentTagDistLateral;
    private double currentTagDistYaw;

    private double currentHubDistForward;
    private double currentHubDistLateral;
    private double currentHubDistYaw;

    private boolean tagVisible = false;
    private LimelightHelpers.RawFiducial[] visibleTags = new LimelightHelpers.RawFiducial[0];

    private double tagAreaPercent;

    /**
     * Creates a new Vision.
     */
    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        //update vision
        tagVisible = LimelightHelpers.getTV(Constants.IDs.LIMELIGHT_NAME);
        visibleTags = LimelightHelpers.getRawFiducials(Constants.IDs.LIMELIGHT_NAME);

        if (tagVisible) {
            //for (LimelightHelpers.RawFiducial tag : LimelightHelpers.getRawFiducials(Constants.IDs.LIMELIGHT_NAME)){
                //if (tag.id == Constants.VisionConstants.HUB_CENTER_TAG_ID){
                    tagPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.IDs.LIMELIGHT_NAME);
                //}
            //}
           
            currentTagDistForward = tagPose[2];
            currentTagDistLateral = tagPose[0];
            currentTagDistYaw = tagPose[4];

            currentHubDistLateral = currentTagDistLateral + Constants.VisionConstants.TAG_TO_HUB_DIST * sin(toRadians(currentTagDistYaw));
            currentHubDistForward = currentTagDistForward + Constants.VisionConstants.TAG_TO_HUB_DIST * cos(toRadians(currentTagDistYaw));

        } else {
            currentTagDistForward = 0;
            currentTagDistLateral = 0;
            currentTagDistYaw = 0;
        }

        SmartDashboard.putNumber("Tag Translation X (forward)", currentTagDistForward);
        SmartDashboard.putNumber("Tag Translation Y (lat)", currentTagDistLateral);
        SmartDashboard.putNumber("Tag ROT Z (yaw)", currentTagDistYaw);

        tagAreaPercent = LimelightHelpers.getTA(Constants.IDs.LIMELIGHT_NAME);
        SmartDashboard.putNumber("Tag Area Percent", tagAreaPercent);

        // the ids of the visible tags (this is just to be posted to smartdashboard)
        double[] ids = new double[visibleTags.length];
        for (int i = 0; i < visibleTags.length; i++) {
            ids[i] = visibleTags[i].id;
        }
        SmartDashboard.putNumberArray("Visible Tag IDs", ids);

    }
    /**
     * resets tagvisible and returns it
     * @return True if there is a tag in sight
     */
    public boolean getTagVisible() {
        return tagVisible = LimelightHelpers.getTV(Constants.IDs.LIMELIGHT_NAME);
    }

    /**
     * 
     * @return a list of all the visible tags (not just their ids but all the info)
     */
    public LimelightHelpers.RawFiducial[] getVisibleTags() {
        return visibleTags;
    }

    /**
     * 
     * @param id the id of the target tag
     * @return the data about the given tag
     */
    public LimelightHelpers.RawFiducial getFiducialById(int id) {
        for (LimelightHelpers.RawFiducial tag : visibleTags) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    /**
     * 
     * @param id the id of the target tag
     * @return True if the tag is visible
     */
    public boolean isTagVisible(int id) {
        return getFiducialById(id) != null;
    }


    /**
     * 
     * @return the directive in the forward direction
     */
    public double getForwardCommand() {
        return forwardCommand;
    }

    /**
     * 
     * @return the directive in the lateral direction
     */
    public double getLateralCommand() {
        return lateralCommand;
    }

    /**
     * 
     * @return the rotational directive
     */
    public double getRotationCommand() {
        return rotationCommand;
    }

    /**
     * 
     * @return the directive required to get to the tag in the forward direction
     */
    public double getTagForward() {
        double ty = LimelightHelpers.getTY(Constants.IDs.LIMELIGHT_NAME);
        double trueForward = (Constants.VisionConstants.HUB_TAG_HEIGHT - Constants.VisionConstants.CAMERA_MOUNT_HEIGHT) / tan(toRadians(Constants.VisionConstants.CAMERA_MOUNT_ANGLE + ty));
        return trueForward;
    }

    /**
     * 
     * @return the directive required to get to the tag in the lateral direction
     */
    public double getTagLateral() {
        return currentTagDistLateral;
    }

    /**
     * 
     * @return the rotational directive required to point to the tag
     */
    public double getTagRotation() {
        return currentTagDistYaw;
    }

    /**
     * 
     * @return the directive required to get to the hub in the forward direction
     */
    public double getHubForward() {
        double yawRad = Math.toRadians(currentTagDistYaw);
        return currentTagDistForward + Constants.VisionConstants.TAG_TO_HUB_DIST * Math.cos(yawRad);
    }

    /**
     * 
     * @return the directive required to get to the hub in the lateral direction
     */
    public double getHubLateral() {
        double yawRad = Math.toRadians(currentTagDistYaw);
        return currentTagDistLateral + Constants.VisionConstants.TAG_TO_HUB_DIST * Math.sin(yawRad);
    }

    /**
     * 
     * @return the rotational directive required to point to the hub
     */
    public double getHubRotation() {
        return Math.toDegrees(Math.atan2(getHubLateral(), getHubForward()));
    }

    /**
     * 
     * @return the percent of the camera view taken up by the tag (used to calculate distance to the tag)
     */
    public double getTagAreaPercent() {
        tagAreaPercent = LimelightHelpers.getTA(Constants.IDs.LIMELIGHT_NAME);
        SmartDashboard.putNumber("Tag Area Percent", tagAreaPercent);
        return tagAreaPercent;
    }

}
