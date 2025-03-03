package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import static edu.wpi.first.units.Units.*;

public final class CTREConfigs {
    public TalonFXConfiguration elevatorFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration armFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration intakeFXConfig = new TalonFXConfiguration();

    public CTREConfigs() {
        /* Elevator Motor */
        var elevatorOutput = elevatorFXConfig.MotorOutput;
        elevatorOutput.Inverted = Constants.ElevatorConstants.ELEVATOR_INVERTED;
        elevatorOutput.NeutralMode = Constants.ElevatorConstants.ELEVATOR_NEUTRAL_MODE;

        /* Elevator Current Limits */
        CurrentLimitsConfigs elevatorCurrentLimits = elevatorFXConfig.CurrentLimits;
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

        // Slot0Configs slot0 = elevatorFXConfig.Slot0;
        // slot0.kG = 0;
        // slot0.kS = 0.25; 
        // slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        // slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0.kP = 15; // A position error of 0.2 rotations results in 12 V output
        // slot0.kI = 0; // No output for integrated error
        // slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

        /* Elevator Motion Magic */
        MotionMagicConfigs elevatorMotionMagic = elevatorFXConfig.MotionMagic;
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


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        


       /* Arm Motor */
       var armOutput = armFXConfig.MotorOutput;
       armOutput.Inverted = Constants.ArmConstants.ARM_INVERTED;
       armOutput.NeutralMode = Constants.ArmConstants.ARM_NEUTRAL_MODE;

       /* Arm Current Limits */
       var armCurrentLimits = armFXConfig.CurrentLimits;
       armCurrentLimits.SupplyCurrentLimitEnable = true;
       armCurrentLimits.SupplyCurrentLimit = 60;
       armCurrentLimits.SupplyCurrentLowerLimit = 40;
       armCurrentLimits.SupplyCurrentLowerTime = 0.5;
       
       /* Arm PID/FF Gains */
       var armSlot1 = armFXConfig.Slot1;
       armSlot1.kG = Constants.ArmConstants.ARM_G;
       armSlot1.kS = Constants.ArmConstants.ARM_S;
       armSlot1.kV = Constants.ArmConstants.ARM_V;
       armSlot1.kA = Constants.ArmConstants.ARM_A;
       armSlot1.kP = Constants.ArmConstants.ARM_P;
       armSlot1.kI = Constants.ArmConstants.ARM_I;
       armSlot1.kD = Constants.ArmConstants.ARM_D;
       
       /* Arm Motion Magic */
       var armMotionMagic = armFXConfig.MotionMagic;
       armMotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(Constants.ArmConstants.ARM_MM_C))
                     .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(Constants.ArmConstants.ARM_MM_A))
                     .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(Constants.ArmConstants.ARM_MM_J));

        /* motor rotation to output of arm*/
        var armfdb = armFXConfig.Feedback; // 200:1 ratio
        armfdb.SensorToMechanismRatio = 200;

        /////////////////////////////////////////////////////////////////////

         /* intake Motor */
       var intakeOutput = intakeFXConfig.MotorOutput;
       intakeOutput.Inverted = Constants.INTAKEConstants.INTAKE_INVERTED;
       intakeOutput.NeutralMode = Constants.INTAKEConstants.INTAKE_NEUTRAL_MODE;

       /* Intake Current Limits */
       var intakeCurrentLimits = intakeFXConfig.CurrentLimits;
       intakeCurrentLimits.SupplyCurrentLimitEnable = true;
       intakeCurrentLimits.SupplyCurrentLimit = 10;
       intakeCurrentLimits.SupplyCurrentLowerLimit = 5;
       intakeCurrentLimits.SupplyCurrentLowerTime = 0.5;
       intakeCurrentLimits.StatorCurrentLimit = 40;
       intakeCurrentLimits.StatorCurrentLimitEnable = true;

    }
}
