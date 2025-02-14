package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public static final class ElevatorConstants {
        // TO-DO: set ids for motor contorllers
        public static final int ELEVATOR_MOTOR_1_ID = 14; 
        public static final int ELEVATOR_MOTOR_2_ID = 15;

        
        public static final InvertedValue ELEVATOR_INVERTED = InvertedValue.Clockwise_Positive; // TO-DO: figure out directions
        public static final NeutralModeValue ELEVATOR_NEUTRAL_MODE = NeutralModeValue.Brake;


        public static final double ELEVATOR_G = 0;
        public static final double ELEVATOR_P = 0;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;
        public static final double ELEVATOR_S = 0;
        public static final double ELEVATOR_V = 0;
        public static final double ELEVATOR_A = 0;

        
    }


}
