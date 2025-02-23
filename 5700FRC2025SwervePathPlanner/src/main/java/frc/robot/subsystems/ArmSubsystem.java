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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.Constants.ElevatorConstants.ElevatorSelector;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID, "rio");
  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ARM_ENCODER_PORT);



  // encoder limit switch 0-180
  // use rev encoder only
  // 

  private MotionMagicVoltage armMM = new MotionMagicVoltage(0);

  public ArmSubsystem() {
    configArmMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //System.out.println("Arm Position: " + getArmPosition());
    //System.out.println("Arm Connected? " + armEncoder.isConnected());
    // System.out.println(armEncoder.get());
  }

  public void configArmMotors() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration()); //this reset to default
    armMotor.getConfigurator().apply(Robot.ctreConfigs.armFXConfig); // arm config

    resetArm();
  }

  public void resetArm() { 
    armMotor.setPosition(1);
  }

  public void setArmPosition(double position) {
    armMM.Position = position;
    armMotor.setControl(armMM.withPosition(position).withSlot(1));
  }
  
  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public void stopArm() {
    armMotor.set(0);
  }

  public void runArmMotor() {
    armMotor.set(0.25);
  }

  public void turn(){
    armMotor.setControl(new NeutralOut()); 
  }
 

}
