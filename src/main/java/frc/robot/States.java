package frc.robot;

public class States {
    //TODO: Stop drivetrain when RobotState is not FREE_DRIVE or SHOOTING
    // (or if we want to allow for movement once aligned, such as moving in an arc such that we stay aligned)
    //Elevator Slow Mode state
    // public static enum ElevatorSlowMode{
    //     LOW(0.15),
    //     HIGH(0.15),
    //     EXTREME(0.15),
    //     NONE(0.15); 

    //     private final double speedFactor;
    //     ElevatorSlowMode(double speedFactor) {
    //         this.speedFactor = speedFactor;
    //     }

    //     public double getSpeedFactor() {
    //         return speedFactor;
    //     }

    // }
    // public static ElevatorSlowMode elevatorSlowMode = ElevatorSlowMode.NONE;

    /////////////////////////////////
    //Intake Position
    public static enum IntakeState{
        UP,
        DOWN
    }

    //States for robot
    public static enum RobotState{
        FREE_DRIVE,
        ALIGNING,
        ALIGNED,
        SHOOTING
    }

    public static RobotState robotState = RobotState.FREE_DRIVE;


    public static IntakeState intakeState = IntakeState.UP;



}
