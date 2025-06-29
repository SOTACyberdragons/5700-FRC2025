package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;

public class Constants {
    
    public static final class ElevatorConstants {
        /* Motor IDs */
        // TO-DO: set ids for motor contorllers
        public static final int ELEVATOR_MOTOR_MASTER_ID = 4; 
        public static final int ELEVATOR_MOTOR_FOLLOW_ID = 3;

        /* Invert + Neutral Mode */
        public static final InvertedValue ELEVATOR_INVERTED = InvertedValue.Clockwise_Positive; // TO-DO: figure out directions
        public static final NeutralModeValue ELEVATOR_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* PID + FF Constants */
        public static final double ELEVATOR_G = 0; //tune until elevator holds postition without input
        public static final double ELEVATOR_S = 0.25; //overcome static friction increase until threashold
        public static final double ELEVATOR_V = 0; //velocity using tuner check steady state velocity and voltage kv = voltage/velocity
        public static final double ELEVATOR_A = 0; //kA = (Voltage - kS - kV * Velocity) / Acceleration
        public static final double ELEVATOR_P = 15;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;

        /* MM Config Constants */
        public static final double ELEVATOR_MM_C = 8; //Cruise
        public static final double ELEVATOR_MM_A = 10; //Acceleration
        public static final double ELEVATOR_MM_J = 100; //Jerk

        /* Elevator Hall Effect 0 point*/

        public static final double ELEVATOR_Hall_Zero = 0; //0 point for elevator


        /* Heights */
        //public static final double ELEVATOR_L1_HEIGHT = 1; 
        public static final double ELEVATOR_L2_HEIGHT = 0.96; //0.96 arm 0.96 elevator 
        public static final double ELEVATOR_L3_HEIGHT = 2.4; //2.23 elevator 0.93 arm
        public static final double ELEVATOR_L4_HEIGHT = 7.07; //7.07 elevator 0.96 arm
        public static final double ELEVATOR_MAX = 7.6;//7.6 elevator


  

    }




    

    public static final class ArmConstants {
        /* Motor IDs */
        // TO-DO: set ids for motor contorllers
        public static final int ARM_MOTOR_ID = 5; 

        /* Invert + Neutral Mode */
        public static final InvertedValue ARM_INVERTED = InvertedValue.Clockwise_Positive; // TO-DO: figure out directions
        public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* PID + FF Constants */
        public static final double ARM_G = 0.12; //tune until elevator holds postition without input
        public static final double ARM_S = 0.05; //overcome static friction increase until threashold
        public static final double ARM_V = 0; //velocity using tuner check steady state velocity and voltage kv = voltage/velocity
        public static final double ARM_A = 0; //kA = (Voltage - kS - kV * Velocity) / Acceleration
        public static final double ARM_P = 400;
        public static final double ARM_I = 0;
        public static final double ARM_D = 0;

        /* MM Config Contants */
        public static final double ARM_MM_C =10; //Cruise
        public static final double ARM_MM_A = 15; //Acceleration
        public static final double ARM_MM_J = 40; //Jerk

        /* Angles */
        public static final double ARM_ANGLE_1 = 0.1;
        public static final double ARM_ANGLE_2 = 0;
        public static final double ARM_ANGLE_3 = 0;
        public static final double ARM_ANGLE_4 = 0;

        public static final int ARM_ENCODER_PORT = 9;

        public static final double ARM_ZERO_POSITION = 0;  //desired arm position encoder value
        public static final double TICKS_PER_REV = 2048.0 * 200.0;  // falcon is 2048 cpr, find out what the motor output to arm is 

        public static final double ARM_ZERO_POSITION_THRESHOLD = 0.001; //0 point threshold

    }
    public static final class VisionConstants {

        public static final double TARGET_LATERAL_OFFSET = 0;


    }
    public static final class INTAKEConstants {

        public static final int INTAKE_MOTOR_ID = 6; 
        public static final InvertedValue INTAKE_INVERTED = InvertedValue.Clockwise_Positive; // TO-DO: figure out directions
        public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;

       public static final int INTAKE_CURRENT_THRESHOLD = 10; 
        public static final int INTAKE_CURRENT_DIR= 5; 

    }




}
