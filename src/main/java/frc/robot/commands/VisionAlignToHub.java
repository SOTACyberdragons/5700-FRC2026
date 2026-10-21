// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;


//TODO: as of right now, this just aligns to whatever tag. the goal would be for it to align differently based on what tag it sees.
public class VisionAlignToHub extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionsubsystem;

    // these pid controllers allow us to control how the robot moves to the target (when it speeds up/slows down and how much)
    private final PIDController rotationPID = new PIDController(VisionConstants.VISION_ROTATION_KP, VisionConstants.VISION_ROTATION_KI, VisionConstants.VISION_ROTATION_KD);
    private final PIDController forwardPID = new PIDController(VisionConstants.VISION_FORWARD_KP, VisionConstants.VISION_FORWARD_KI, VisionConstants.VISION_FORWARD_KD);
    private final PIDController lateralPID = new PIDController(VisionConstants.VISION_LATERAL_KP, VisionConstants.VISION_LATERAL_KI, VisionConstants.VISION_LATERAL_KD);

    // this drive request allows us to control the drivetrain
    private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    // thses directives will later be passed to the visionRequest to make the robot drive
    private double forwardCommand;
    private double lateralCommand;
    private double rotationCommand;

    // this is the current joystick input. we only use lateral as of right now
    private Supplier<Double> forwardSupplier;
    private Supplier<Double> lateralSupplier;
    private Supplier<Double> rotationSupplier;


    public VisionAlignToHub(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionsubsystem,
            Supplier<Double> forwardSupplier,
            Supplier<Double> lateralSupplier,
            Supplier<Double> rotationSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.lateralSupplier = lateralSupplier;
        this.rotationSupplier = rotationSupplier;
        this.drivetrain = drivetrain;
        this.visionsubsystem = visionsubsystem;

        addRequirements(drivetrain, visionsubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("vision cmd");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if we can see a tag
        //  && visionsubsystem.isTagVisible(VisionConstants.BEST_TAG_ID)
        if (visionsubsystem.getTagVisible()) {
            //TODO: use the best tag if possible
            // create a directive to move forward by the difference between (the distance between the hub and the bot) and (the shoot distance setpoint)
            forwardCommand = forwardPID.calculate(
                visionsubsystem.getHubForward(), 
                VisionConstants.LONG_SHOOT_DIST_TO_HUB
            );
            // create a directive to rotate by the difference between the hub's rotation and 0 (aka rotate to the hub's rotation)
            rotationCommand = rotationPID.calculate(
                visionsubsystem.getHubRotation(), 
                0
            );

        } else {
            forwardCommand = 0;//lateralSupplier.get();
            rotationCommand = 0;//rotationSupplier.get();
        }

        // due to pid error, we will never be exactly on target, so we stop worrying about it when we're close enough
        // check if the rotation to the hub relative to the bot is less than our constant
        // if so, stop worrying about it
        // if (abs(visionsubsystem.getHubRotation()) >= VisionConstants.CLOSE_ENOUGH_ROT) {
        //     rotationCommand = 0;
        // }
        
        // // do the same with forward
        // if (abs(visionsubsystem.getHubForward()) >= VisionConstants.CLOSE_ENOUGH_FOR) {
        //     forwardCommand = 0;
        // }



        // still drive with lateral, but get it from the joystick (aka vision doesn't control lat)
        lateralCommand = lateralSupplier.get();

        System.out.println(forwardCommand);
        System.out.println(rotationCommand);
        // this part does the actual driving, using the previously determined commands in place of joystick input
        drivetrain.setControl(
                visionRequest.withVelocityX(forwardCommand * VisionConstants.VISION_SPEED_MULTIPLIER)
                        .withVelocityY(-lateralCommand) //FIXME: if you want to slow our sotm multiply this by the speed multiplier
                        .withRotationalRate(rotationCommand * VisionConstants.VISION_SPEED_MULTIPLIER)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //     // Stop the drivetrain when the command ends
        // drivetrain.setControl(
        //     visionRequest.withVelocityX(0)
        //                  .withVelocityY(0)
        //                  .withRotationalRate(0)
        // );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
