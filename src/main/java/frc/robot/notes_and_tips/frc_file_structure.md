# FRC File Structure
## File and folder structure
```
.gradle
.vscode
.wpilib
bin
build
gradle
src
    main
        deploy
        java
            frc
                robot
                    commands
                        AutoCMDs
                            ExampleCMD.java
                    generated
                        TunerConstants.java
                    subsystems
                        ExampleSubsystem.java
                    Constants.java
                    CTREConfigs.java
                    Main.java
                    Robot.java
                    RobotContainer.java
```
## Subsystems
In the `subsystems` folder, you will be listing all of the subsystems on the robot as individual files. The `subsystems` folder should contain a `CommandSwerveDrivetrain` file in it. Each file should look generally like this:
```java
public class ExampleSubsystem extends SubsystemBase {
    // Declare motors and sensors here. They can be declared like this: 
    private TalonFX exampleMotor = new TalonFX(Constants.IDs.EXAMPLE_MOTOR_ID) // Motor

    private DigitalInput exambleSeneor; // Sensor

    /** Creates a new ExampleSubsystem. */
    public ExampleSubsystem() {
        configureExampleSubsystem();
    }

    public void configureExampleSubsystem() {
        // Configure motors. Do this for each motor.
        motor.getConfigurator()
            .apply(new TalonFXConfiguration()); // Change to TalonSRXConfiguration if using TalonSRX.
        motor.getConfigurator()
            .apply(Robot.ctreConfigs.exampleFXConfig);

        // Configure sensors. Do this for each sensor. 
        exampleSensor = new DigitalInput(Constants.Ids.EXAMPLE_SENSOR_ID);
    }
    public void exampleMethod() {
        // Do something, such as run a certain motor. See motor_control.md for more info on how to do that.
    }
    // Declare getter functions for sensors
    public boolean getExampleSensor() {
        return !exampleSensor.get(); // Not sure why this is flipping the result; if something seems wrong, this is why
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Many times, it will be usefil to add extra stuff to the dashboard for testing.
        SmartDashboard.putNumber('exampleSensor value', getExampleSensor());
    }
}

```
## Commands
In the `commands` folder, you will list all of the things thaat your robot can do, such as intaking a game piece, moving around using the drivetrain, moving an elevator or arm, or scoring a game piece. Each file should look generally like this:
```java
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExampleCommand extends Command {
    // Declare subsystems and other general variables here
    private final ExampleSubsystem m_exampleSubsystem;

    /** Creates a new ExampleCommand. */
    public ExampleCommand(ExampleSubsystem subsystem) {
        this.m_exampleSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // this theoretically would run forever if you just called it normally, but for any commands that are called using a trigger binding, we make them run "forever," which ends up actually just being until the driver releases the button
    }
}
```
## AutoCMDs
For each command that you plan on using during auto (usually everything but the climb, just in case), you should make that file in the `AutoCMDs` folder. Each file should look generally like this:
```java
public class ExampleCMD extends Command {
    private ExampleSubsystem m_exampleSubsystem;
    private boolean isKilled = false;
    /**
     * Creates a new ExampleCMD
     */

    public ExampleCMD(ExampleSubsystem subsystem) {
        this.m_exampleSubsystem = subsystem;
        // Add subsystem requirements here if needed
        // addRequirements(subsystem);
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Add initialization code here
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        // Add execution code here
        // Change isKilled to true based on logic here
    }

    /**
     * Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // Add cleanup code here
    }

    /**
     * Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return isKilled; // since this is an autoCMD, we use an isKilled variable to determine when the command should end, such as when a sensor detects that a piece has been intaked.
    }
}
```
## Constants.java
Your `Constants.java` file should contain all of the constants that you use throughout the robot code. **<u>NEVER EVER EVER</u>** just hard-code a number into the robot code. **<u>ALWAYS ALWAYS ALWAYS</u>** put it into the `Constants.java` file. Even if the number is "just a test" or you're on a time crunch, there is never harm in putting it into the `Constants.java` file. This makes debugging, testing, and calibration easier and faster, because all of the variables that you might need to change are in the same place. Each constant should be prefixed by `public static final` This is what the structure of a good `Constants.java` file looks like:
```java
public final class Constants {
    public static class OperatorConstants {
        public static final int k_DRIVER_CONTROLLER_PORT = 0;
    }

    public static class IDs {
        public static final int EXAMPLE_MOTOR_1_ID = 0; // format these as: subsystem_thing_number_ID
    }

    public static class ExampleSubsystemConstants {
        public static final int EXAMPLE_CONSTANT = 0;

    }
}

```
## Main.java
**<u>DO NOT CHANGE THIS FILE EVER NO MATTER WHAT SOMEONE TELLS YOU NO MATTER WHAT YOU SEE ON THE INTERNET NEVER CHANGE THIS FILE!!!</u>** It should look like this:
```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
```
## Robot.java
Although changing this file will probably not break everything, do not just add extra stuff to this file on your own. This file is auto-generated, and some packages will request that you edit it, but beyond that, leave it alone. It should look like this:
```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
```
## RobotContainer.java
This file is totally OK to edit, and in fact much of the code for the robot will be in this file. You will be configuring the trigger bindings in this file, as well as setting up the auto commands. LEDs and ther extras can also go here. It should look generally like this:
```java

```