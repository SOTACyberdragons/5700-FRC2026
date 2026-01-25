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
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.LoggableRobotPose;
import frc.robot.vision.PhotonVisionSystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSetpoint;
import frc.robot.subsystems.ShooterSubsystem.FlywheelSetpoint;



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
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.DrivetrainConstants.MAX_ANGULAR_RATE).in(RadiansPerSecond);

	// limits the change in the drivetrain; makes sure that we don't make any sharp turns. remove if not a concern
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.SKEW_RATE_LIMITER_Y); 
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.SKEW_RATE_LIMITER_X); 
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.SKEW_RATE_LIMITER_ROTATION); 

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(10, 0, 0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // The robot's subsystems and commands are defined here...
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public IntakeSubsystem m_intake = new IntakeSubsystem();
    public ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public final PhotonVisionSystem vision = new PhotonVisionSystem(this::consumePhotonVisionMeasurement, () -> drivetrain.getState().Pose);

    // path follower
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.k_DRIVER_CONTROLLER_PORT);

    private final AngularVelocity SpinUpThreshold = RotationsPerSecond.of(ShooterConstants.SPINUP_THRESHOLD); // Tune to increase accuracy while not sacrificing throughput
    private final Trigger isFlywheelReadyToShoot = m_shooterSubsystem.getTriggerWhenNearTargetVelocity(SpinUpThreshold).or(joystick.x());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // register all autoCMDs here
        NamedCommands.registerCommand("Stop Shooting", m_shooterSubsystem.coastFlywheel().alongWith(m_intake.coastIntake()));
        /* Shoot commands need a bit of time to spool up the flywheel before feeding with the intake */
        NamedCommands.registerCommand("Shoot Near", m_shooterSubsystem.setTarget(() -> FlywheelSetpoint.Near)
                                                            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(m_intake.setTarget(() ->IntakeSetpoint.FeedToShoot))));
        NamedCommands.registerCommand("Shoot Far", m_shooterSubsystem.setTarget(() -> FlywheelSetpoint.Far)
                                                            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(m_intake.setTarget(() ->IntakeSetpoint.FeedToShoot))));
        NamedCommands.registerCommand("Stop Intake", m_intake.coastIntake().alongWith(m_shooterSubsystem.coastFlywheel()));
        NamedCommands.registerCommand("Intake Fuel", m_intake.setTarget(() -> IntakeSetpoint.Intake).alongWith(m_shooterSubsystem.setTarget(()-> FlywheelSetpoint.Intake)));
        NamedCommands.registerCommand("Outtake Fuel", m_intake.setTarget(() -> IntakeSetpoint.Outtake).alongWith(m_shooterSubsystem.setTarget(()-> FlywheelSetpoint.Outtake)));
        // auto stuff
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Configure the trigger bindings
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
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
                    /* Use the hub target to determine where to aim TODO: maybe point the wheels so that they are perpendicular to the hub? Do this only if we go with a non-variable shooter*/
                    return targetHub.withTargetDirection(vision.getHeadingToHubFieldRelative())
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed); // Drive left with negative X (left)
                }
            }
        ));

        /* TODO: would be really cool to add a button for going over the bump. it would be a hold that 
            * gets the current rotation,
            * finds which of the 4 directions it is closest to (forward, back, left, right),
            * turns the robot 45deg from that rotation.
            * and then limits the sped to the exact amount we need to go over the bump

        I would suggest using B for this
        */ 
        
        /*Drive robot centric */
        /* this code outputs a flat amount of movement while driving robot centric, 
        so it drives really slowly. this is used for small adjustments or alignments. 
        Depending on the game, this may or may not be useful.
        */

        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0).withVelocityY(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0).withVelocityY(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED))
        );
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(Constants.DrivetrainConstants.ROBOT_CENTRIC_DRIVE_SPEED).withVelocityY(0))
        );

		// reset the field centric position in case the robot becomes misaligned
		joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /////// Shooting and intaking
        // A (toggle) -> pivot intake up or down
        joystick.a().onTrue(m_intake.togglePivot());

        // Left Bumper (hold) -> Intake
        joystick.leftBumper().whileTrue(
            m_intake.setTarget(()->IntakeSetpoint.Intake) // intake
            .alongWith(m_shooterSubsystem.setTarget(()->FlywheelSetpoint.Intake)) // this is feeding into the shooter?
        );
        // Left Trigger (hold) -> Outtake all
        joystick.leftTrigger().whileTrue(
            m_intake.setTarget(()->IntakeSetpoint.Outtake) //outtake
            .alongWith(m_shooterSubsystem.setTarget(()->FlywheelSetpoint.Outtake)) // also outtake shooter
        );

        // Right bumper (hold) -> Shoot(near)
        joystick.rightBumper().whileTrue(
            m_shooterSubsystem.setTarget(()->FlywheelSetpoint.Near) // First spin up the flywheel
            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot) // wait until ready to shoot
            .andThen(m_intake.setTarget(()->IntakeSetpoint.FeedToShoot))) // use the intake to push balls into the shooter
        );
        // Right trigger (hold) -> Shoot(far)
        joystick.rightTrigger().whileTrue(
            m_shooterSubsystem.setTarget(()->FlywheelSetpoint.Far) // First spin up the flywheel
            .alongWith(Commands.waitUntil(isFlywheelReadyToShoot) // wait until ready to shoot
            .andThen(m_intake.setTarget(()->IntakeSetpoint.FeedToShoot))) // use the intake to push balls into the shooter
        );

        // X (press) -> override isReadyToShoot (see ln 92)

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