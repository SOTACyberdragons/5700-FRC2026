package frc.robot.commands.AutoCMDs;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Example autonomous command that does nothing.
 * This is a template for creating new commands.
 */
public class ShooterCMD extends Command {
    
    /**
     * Creates a new ExampleCMD
     */

    private boolean killed = false;

    public ShooterCMD() {
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
        return killed;
    }
}

