// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// frc imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // The robot's subsystems and commands are defined here...
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // path follower
    private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController joystick = new CommandXboxController(
        OperatorConstants.k_DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // register all autoCMDs here
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
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        //         .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
        
        /*Drive robot centric */
        /* this code outputs a flat amount of movement while driving robot centric, 
        so it drives really slowly. this is used for small adjustments or alignments. 
        Depending on the game, this may or may not be useful.
        */
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0).withVelocityY(-0.5))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0).withVelocityY(0.5))
        );
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0.5).withVelocityY(0))
        );

		// reset the field centric position in case the robot becomes misaligned
		joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
}
