package frc.robot;

public class States {
    /*
    public enum ElevatorSelector {
        RESET(0),
        L1(ELEVATOR_L1_HEIGHT),
        L2(ELEVATOR_L2_HEIGHT),
        L3(ELEVATOR_L3_HEIGHT),
        L4(ELEVATOR_L4_HEIGHT);

        private double height;

        private ElevatorSelector(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    } */
    /////////////////////////////////
    //Elevator Slow Mode state
    public static enum ElevatorSlowMode{
        LOW(0.5),
        HIGH(0.1),
        EXTREME(0.05),
        NONE(0.8); 

        private final double speedFactor;
        ElevatorSlowMode(double speedFactor) {
            this.speedFactor = speedFactor;
        }

        public double getSpeedFactor() {
            return speedFactor;
        }

    }
    public static ElevatorSlowMode elevatorSlowMode = ElevatorSlowMode.NONE;

    /////////////////////////////////
    //Intake Direction
    public static enum IntakeDirection{
        CORAL,
        ALGAE,
        NONE
    }
    public static IntakeDirection intakeState = IntakeDirection.NONE;

    //States for arm and elevator
    public static enum ElevatorState{
        GROUND,
        L2CLEARED,
        STAGE1CLEARED,
        NOTCLEAR
        
    }
    public static ElevatorState elevatorState = ElevatorState.GROUND;

    public static enum ArmState{
        START,
        CLEAR,
        INTAKE,
        SCORE
        
    }
    public static ArmState armState = ArmState.START;



}
