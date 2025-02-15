package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public static final class ElevatorConstants {
        /* Motor IDs */
        // TO-DO: set ids for motor contorllers
        public static final int ELEVATOR_MOTOR_MASTER_ID = 14; 
        public static final int ELEVATOR_MOTOR_FOLLOW_ID = 15;

        /* Invert + Neutral Mode */
        public static final InvertedValue ELEVATOR_INVERTED = InvertedValue.Clockwise_Positive; // TO-DO: figure out directions
        public static final NeutralModeValue ELEVATOR_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* PID + FF Constants */
        public static final double ELEVATOR_G = 0;
        public static final double ELEVATOR_S = 0;
        public static final double ELEVATOR_V = 0;
        public static final double ELEVATOR_A = 0;
        public static final double ELEVATOR_P = 0;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;

        /* Heights */
        public static final double ELEVATOR_L1_HEIGHT = 0;
        public static final double ELEVATOR_L2_HEIGHT = 0;
        public static final double ELEVATOR_L3_HEIGHT = 0;
        public static final double ELEVATOR_L4_HEIGHT = 0;
    }


}
