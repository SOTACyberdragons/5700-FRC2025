// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants.ElevatorSelector;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX armMasterMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_MASTER_ID);
  private TalonFX armFollowMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_FOLLOW_ID);
  private Encoder encoder = new Encoder(Constants.ArmConstants.ARM_ENCODER_CHANNELA, Constants.ArmConstants.ARM_ENCODER_CHANNELB);

  private MotionMagicVoltage armMM = new MotionMagicVoltage(0);

  public ArmSubsystem() {
    configArmMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //System.out.println("Elevator Position: " + getElevatorPosition());
  }

  public void configArmMotors() {
    armMasterMotor.getConfigurator().apply(new TalonFXConfiguration()); //this reset to default
    armMasterMotor.getConfigurator().apply(Robot.ctreConfigs.armFXConfig); //elevator config

    armFollowMotor.getConfigurator().apply(new TalonFXConfiguration());
    armFollowMotor.getConfigurator().apply(Robot.ctreConfigs.armFXConfig);

    armFollowMotor.setControl(new Follower(armMasterMotor.getDeviceID(), true));

    resetArm();
  }

  public void resetArm() { 
    armMasterMotor.setPosition(0);
    armFollowMotor.setPosition(0);
  }

  public void setArmPosition(ElevatorSelector levelChoice) {
    double setpoint = levelChoice.getHeight();
    armMM.Position = setpoint;
    armMasterMotor.setControl(armMM.withPosition(setpoint).withSlot(0));
  }
  
  public double getElevatorPosition() {
    return armMasterMotor.getPosition().getValueAsDouble();
  }

  public void stopElevator() {
    armMasterMotor.set(0);
    armFollowMotor.set(0);
  }

  public void turn(){
    armMasterMotor.setControl(new NeutralOut()); 
    armFollowMotor.setControl(new NeutralOut()); 
  }
 

}
