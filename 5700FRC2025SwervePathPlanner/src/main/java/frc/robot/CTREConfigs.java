package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import static edu.wpi.first.units.Units.*;

public class CTREConfigs {
    public static TalonFXConfiguration elevatorFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration armFXConfig = new TalonFXConfiguration();

    public CTREConfigs() {
        /* Elevator Motor */
        var elevatorOutput = elevatorFXConfig.MotorOutput;
        elevatorOutput.Inverted = Constants.ElevatorConstants.ELEVATOR_INVERTED;
        elevatorOutput.NeutralMode = Constants.ElevatorConstants.ELEVATOR_NEUTRAL_MODE;

        /* Elevator Current Limits */
        var elevatorCurrentLimits = elevatorFXConfig.CurrentLimits;
        elevatorCurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorCurrentLimits.SupplyCurrentLimit = 60;
        elevatorCurrentLimits.SupplyCurrentLowerLimit = 40;
        elevatorCurrentLimits.SupplyCurrentLowerTime = 0.5;

        /* Elevator PID/FF Gains */
        var elevatorSlot0 = elevatorFXConfig.Slot0;
        elevatorSlot0.kG = Constants.ElevatorConstants.ELEVATOR_G;
        elevatorSlot0.kS = Constants.ElevatorConstants.ELEVATOR_S;
        elevatorSlot0.kV = Constants.ElevatorConstants.ELEVATOR_V;
        elevatorSlot0.kA = Constants.ElevatorConstants.ELEVATOR_A;
        elevatorSlot0.kP = Constants.ElevatorConstants.ELEVATOR_P;
        elevatorSlot0.kI = Constants.ElevatorConstants.ELEVATOR_I;
        elevatorSlot0.kD = Constants.ElevatorConstants.ELEVATOR_D;

        /* Elevator Motion Magic */
        var elevatorMotionMagic = elevatorFXConfig.MotionMagic;
        elevatorMotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(Constants.ElevatorConstants.ELEVATOR_MM_C))
                           .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(Constants.ElevatorConstants.ELEVATOR_MM_A))
                           .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(Constants.ElevatorConstants.ELEVATOR_MM_J));
        
        /* motor rotation to output of elevator*/
        var elevatorfdb = elevatorFXConfig.Feedback; // 10:1 ratio
        elevatorfdb.SensorToMechanismRatio = 10;
        
        /*Hall Effect Config */
        var elevatorHallEffect = elevatorFXConfig.HardwareLimitSwitch;
        elevatorHallEffect.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        elevatorHallEffect.ForwardLimitAutosetPositionValue = Constants.ElevatorConstants.ELEVATOR_Hall_Zero;
        elevatorHallEffect.ForwardLimitAutosetPositionEnable = true;


    }
}
