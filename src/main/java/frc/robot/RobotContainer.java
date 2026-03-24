// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// frc imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederPercentSetpoint;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FeederPercentCommand;
import frc.robot.commands.IntakePercentCommand;
import frc.robot.commands.IntakePercentDefaultCommand;
import frc.robot.commands.OuttakePercentCommand;
import frc.robot.commands.ShooterPercentDefaultCommand;
import frc.robot.commands.ShooterPercentSetpointCommand;
import frc.robot.commands.SmartShootVelocityCommand;
import frc.robot.commands.AutoCMDs.IntakeCMD;
import frc.robot.commands.AutoCMDs.OuttakeCMD;


import frc.robot.Constants.ShooterPercentSetpoint;
import frc.robot.Constants.ShooterVelocitySetpoint;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.LoggableRobotPose;
import frc.robot.vision.PhotonVisionSystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    /* Drive variables */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DrivetrainConstants.MAX_SPEED_MULTIPLIER; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.DrivetrainConstants.MAX_ANGULAR_RATE).in(RadiansPerSecond);

	// limits the change in the drivetrain; makes sure that we don't make any sharp turns. remove if not a concern
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.SKEW_RATE_LIMITER_Y); 
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.SKEW_RATE_LIMITER_X); 
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.SKEW_RATE_LIMITER_ROTATION); 

    //// variables for controlling driving of the robot. these are called using various functions such as drivetrain.applyRequest(fieldCentricDrive)
    // drive based onthe robots rotation and position on the field
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    // x-lock control
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // point at a specific direction
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // vision-constricted driving
    private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(Constants.DrivetrainConstants.K_P_HUB_CENTRIC, Constants.DrivetrainConstants.K_I_HUB_CENTRIC, Constants.DrivetrainConstants.K_D_HUB_CENTRIC)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    // robot-centric driving
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // The robot's subsystems are defined here...
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
    public final PhotonVisionSystem vision = new PhotonVisionSystem(this::consumePhotonVisionMeasurement, () -> drivetrain.getState().Pose);
    public final LED m_led = new LED();


    


    // path follower
    private final SendableChooser<Command> autoChooser;

    // represents the controller
    private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.k_DRIVER_CONTROLLER_PORT);

    // public final Trigger isFlywheelReadyToShoot = m_shooterSubsystem.getTriggerWhenNearTargetVelocity(SpinUpThreshold).or(joystick.x());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // register all autoCMDs here
        // NamedCommands.registerCommand("Coast All", 
        //     m_intakeSubsystem.coastIntake()
        //     .alongWith(m_shooterSubsystem.coastFlywheel())
        // );

        // TODO: instead of coasting, just move at small voltage

        NamedCommands.registerCommand("Intake", 
            new IntakeCMD(m_intakeSubsystem)
            .alongWith(new FeederPercentCommand(m_feederSubsystem, false, FeederPercentSetpoint.Intake))
        );

        NamedCommands.registerCommand("Outtake", 
            new OuttakeCMD(m_intakeSubsystem)
            .alongWith(new ShooterPercentSetpointCommand(m_shooterSubsystem, ShooterPercentSetpoint.Outtake))
        );

        NamedCommands.registerCommand("Target Hub", 
            Commands.runOnce(() -> targetHub.withTargetDirection(vision.getHeadingToHubFieldRelative()))
        );

        // auto stuff
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // initialize booleans for dashboard
        SmartDashboard.putBoolean("Intaking", false);
        SmartDashboard.putBoolean("Outtaking", false);
        SmartDashboard.putBoolean("Shooting", false);
        SmartDashboard.putBoolean("Braking", false);



        // Configure the trigger bindings
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        /* /// DRIVETRAIN /// */
		// Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            fieldCentricDrive
                .withVelocityX(
                    xLimiter.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(
                    yLimiter.calculate(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(
                    rotLimiter.calculate(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        
        // set default command for subsystems. this is what the subsystem does if not commanded to do something else
        m_intakeSubsystem.setDefaultCommand(new IntakePercentDefaultCommand(m_intakeSubsystem));
        m_shooterSubsystem.setDefaultCommand(new ShooterPercentDefaultCommand(m_shooterSubsystem));
        m_feederSubsystem.setDefaultCommand(m_feederSubsystem.coastFeeder());


        ///// Alternate driving
        /* Y (hold) -> Vision-constricted driving */
        joystick.y().whileTrue(
            drivetrain.applyRequest(()-> {
                if (!vision.isHubTargetValid()) {
                    /* Do typical field-centric driving since we don't have a target */
                    return fieldCentricDrive
                        .withVelocityX(
                            xLimiter.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            yLimiter.calculate(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            rotLimiter.calculate(-joystick.getRightX()) * MaxAngularRate); // Drive counterclockwise with negative X (left)
                } else {
                    SmartDashboard.putBoolean("Vision Activated", true);
                    /* Use the hub target to determine where to aim TODO: maybe point the wheels so that they are perpendicular to the hub? Do this only if we go with a non-variable shooter*/
                    return targetHub.withTargetDirection(vision.getHeadingToHubFieldRelative())
                        .withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(joystick.getLeftX() * MaxSpeed); // Drive2``````` left with negative X (left)
                }
            }
        ));
        // reset "vision activated" boolean on the smartdashboard when we stop driving with vision
        joystick.y().onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Activated", false)));

        // x-lock, aka brake
        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> brake)
            .alongWith(Commands.runOnce(()->SmartDashboard.putBoolean("Braking", true)))
            );

        joystick.a().onFalse(Commands.runOnce(()->SmartDashboard.putBoolean("Braking", false)));
        
        /*Drive robot centric */
        /* this code outputs a flat amount of movement while driving robot centric, 
        so it drives really slowly. this is used for small adjustments or alignments. 
        Depending on the game, this may or may not be useful.
        */

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(0))
        );

        joystick.pov(45).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );

        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0).withVelocityY(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );

        joystick.pov(135).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );

        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(0))
        );

        joystick.pov(225).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );

        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0).withVelocityY(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );

        joystick.pov(315).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );

		// reset the field centric position in case the robot becomes misaligned
		joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /////// Shooting and intaking

        // Left Trigger (hold) -> Intake
        joystick.leftTrigger().whileTrue(
            new IntakePercentCommand(m_intakeSubsystem)
            .alongWith(new FeederPercentCommand(m_feederSubsystem, false, FeederPercentSetpoint.Intake))
        );

        
        // Left Bumper (hold) -> Outtake intake
        joystick.leftBumper().whileTrue(
            new OuttakePercentCommand(m_intakeSubsystem)
            .alongWith(new FeederPercentCommand(m_feederSubsystem, false, FeederPercentSetpoint.Outtake))
        );
        
        
        // Right bumper (hold) -> Shoot(near)
        joystick.rightBumper().whileTrue(
            // new ShooterPercentSetpointCommand(m_shooterSubsystem, ShooterPercentSetpoint.Near)
            // .alongWith(new FeederPercentCommand(m_feederSubsystem, FeederPercentSetpoint.Feed))
            new SmartShootVelocityCommand(m_shooterSubsystem, m_feederSubsystem, ShooterVelocitySetpoint.Near)
        );

        // Right trigger (hold) -> Shoot(far)
        joystick.rightTrigger().whileTrue(
            // new ShooterPercentSetpointCommand(m_shooterSubsystem, ShooterPercentSetpoint.Far)
            // .alongWith(new FeederPercentCommand(m_feederSubsystem, FeederPercentSetpoint.Feed))
            new SmartShootVelocityCommand(m_shooterSubsystem, m_feederSubsystem, ShooterVelocitySetpoint.Far)
        );

        // X (hold) -> Run feeder in (useful for agitation)
        joystick.x().whileTrue(
            new FeederPercentCommand(m_feederSubsystem, false, FeederPercentSetpoint.Intake)
        );
        // B (hold) -> Run feeder out (useful for depositing to human player)
        joystick.b().whileTrue(
            new FeederPercentCommand(m_feederSubsystem, false, FeederPercentSetpoint.Outtake)
        );

    }



  

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        return autoChooser.getSelected();
    }

        public void consumePhotonVisionMeasurement(LoggableRobotPose pose) {
        /* Super simple, should modify to support variable standard deviations */
        drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    public void periodic() {
        vision.periodic();

    }

    public void simulationPeriodic() {
        var drivetrainPose = drivetrain.m_simOdometry.getPoseMeters();
        vision.simPeriodic(drivetrainPose);

        var debugField = vision.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrainPose);
    }
}