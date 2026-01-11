// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;


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

  private double lateralCommandL;
  private double lateralCommandR;

  private double[] targetPose = new double[6];
  private double[] targetPose2 = new double[6];
  
  private double currentForward;
  private double currentLateral;
  private double currentYaw;

  private double currentForward2;
  private double currentLateral2;
  private double currentYaw2;

  private boolean targetVisibleLL1 = false;
  private boolean targetVisibleLL2 = false;

  private double ta1;
  private double ta2;



  /** Creates a new Vision. */
  public VisionSubsystem() {
    
  }

  @Override
  public void periodic() {
   targetVisibleLL1 = LimelightHelpers.getTV("limelight-left"); // Valid target flag
   targetVisibleLL2 = LimelightHelpers.getTV("limelight-right"); // Valid target flag




    /* 
    if (currentYaw == -180 || currentYaw == 180) {
      currentYaw = 0;
    } else if (currentYaw > 0) {
      currentYaw -= 180;
    } else if (currentYaw < 0) {
      currentYaw += 180;
    }
*/
    //if (currentYaw < 0.25 || currentYaw > -0.25) currentYaw = 0;

    // LEFT: Forward: 0.477; Lateral: 0.164


    // if (targetVisible) {
    //   // Calculate PID commands for forward, lateral, and rotation
    //   botPose = LimelightHelpers.getTargetPose_CameraSpace("limelight"); // Get robot pose in world frame
    
    //   currentForward = botPose[2]; // Forward distance
    //   currentLateral = botPose[0]; // Lateral offset
    //   currentYaw = botPose[4];

    //   forwardCommand = forwardPID.calculate(currentForward, 0.7); // Target forward distance (1 meter away) 0.477
    //   lateralCommand = lateralPID.calculate(currentLateral);
    //   rotationCommand = rotationPID.calculate(currentYaw, 0);

    //   lateralCommandL = lateralPIDL.calculate(currentLateral, 0.167); // 0.01651
    //   lateralCommandR = lateralPIDR.calculate(currentLateral, -0.167);
    // }else{
    //   botPose[0] = 0.0;
    //   botPose[1] = 0.0;
    //   botPose[2] = 0.0;
    //   botPose[3] = 0.0;
    //   botPose[4] = 0.0;
    //   botPose[5] = 0.0;

    //   forwardCommand = 0;
    //   lateralCommand = 0;
    //   rotationCommand = 0;

    //   lateralCommandL = 0;
    //   lateralCommandR = 0;
    // }
    if (targetVisibleLL1) {
  double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace("limelight-left");
  // Translation3d translation = targetPose.getTranslation();
  // Rotation3d rotation = targetPose.getRotation();

  currentForward = targetPose[2]; // Assuming x is forward
  currentLateral = targetPose[0]; // Assuming y is lateral
  currentYaw = targetPose[4];
  } else{
    currentForward = 0; 
    currentLateral = 0;
    currentYaw = 0;
  }

  if (targetVisibleLL2) {
    double[] targetPose2 = LimelightHelpers.getTargetPose_CameraSpace("limelight-right");

  
    currentForward2 = targetPose2[2]; // Assuming x is forward
    currentLateral2 = targetPose2[0]; // Assuming y is lateral
    currentYaw2 = targetPose2[4];
    }else{
      currentForward2 = 0; 
      currentLateral2 = 0;
      currentYaw2 = 0;
    }

    SmartDashboard.putNumber("Translation X", currentForward);
    SmartDashboard.putNumber("Translation Y", currentLateral);
    SmartDashboard.putNumber("ROT Z", currentYaw);

    SmartDashboard.putNumber("Translation2 X", currentForward2);
    SmartDashboard.putNumber("Translation2 Y", currentLateral2);
    SmartDashboard.putNumber("ROT2 Z", currentYaw2);
    ta1 = LimelightHelpers.getTA("limelight-left"); 
    SmartDashboard.putNumber("ta1", ta1);
    ta2 = LimelightHelpers.getTA("limelight-right"); 
    SmartDashboard.putNumber("ta2", ta2);
    
  }

  public boolean getTargetVisibleLL1() {
    return targetVisibleLL1 = LimelightHelpers.getTV("limelight-left");
   }

   public boolean getTargetVisibleLL2() {
    return targetVisibleLL2 = LimelightHelpers.getTV("limelight-right");
   }




  public double getForwardCommand() {
    return forwardCommand;
  }

  public double getLateralCommand() {
    return lateralCommand;
  }

  public double getLateralCommandL() {
    return lateralCommandL;
  }

  public double getLateralCommandR() {
    return lateralCommandR;
  }

  public double getRotationCommand() {
    return rotationCommand;
  }

  // public boolean targetReached() {
  //   if (((Math.abs(botPose[2] - 0.05)) < 0.01) && 
  //       ((Math.abs(botPose[0])) < 0.01) &&
  //       ((Math.abs(botPose[4])) < 1)) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  public double getForward() {
    return currentForward;
  }

public double getLateral() {
  return currentLateral;
}

public double getRotation() {
  return currentYaw;
}

public double getForward2() {
  return currentForward2;
}

public double getLateral2() {
return currentLateral2;
}

public double getRotation2() {
return currentYaw2;
}

public double getTA1(){
  ta1 = LimelightHelpers.getTA(Constants.VisionConstants.APRIL_TAG_1_NAME); 
  SmartDashboard.putNumber("ta1", ta1);
  return ta1;
}
public double getTA2(){
  ta2 = LimelightHelpers.getTA(Constants.VisionConstants.APRIL_TAG_2_NAME); 
  SmartDashboard.putNumber("ta2", ta2);
  return ta2;
}

}