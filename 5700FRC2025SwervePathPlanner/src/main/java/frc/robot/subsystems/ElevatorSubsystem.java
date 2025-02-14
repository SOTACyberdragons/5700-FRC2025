// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX elevatorMotor1 = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID);
  private TalonFX elevatorMotor2 = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID);

  private MotionMagicVoltage elevatorMM = new MotionMagicVoltage(0);

  public ElevatorSubsystem() {
    configElevatorMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configElevatorMotor() {
    elevatorMotor1.getConfigurator().apply(new TalonFXConfiguration());
    elevatorMotor1.getConfigurator().apply(CTREConfigs.elevatorFXConfig);

    elevatorMotor2.getConfigurator().apply(new TalonFXConfiguration());
    elevatorMotor2.getConfigurator().apply(CTREConfigs.elevatorFXConfig);

    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

    resetElevatorEncoder();
  }

  public void resetElevatorEncoder() {
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);
  }

  public void setElevatorPosition(double setpoint) {
    elevatorMM.Position = setpoint;
    elevatorMotor1.setControl(elevatorMM.withPosition(setpoint));
  }
  
  public double getElevatorPosition() {
    return elevatorMotor1.getPosition().getValueAsDouble();
  }
}
