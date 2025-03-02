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


public class ArmSubsystem extends SubsystemBase {

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
    /*
    if(isAtZeroPosition()){
      resetArm();
    }
    */
    System.out.println("Arm Position: " + getArmPosition());
    //System.out.println("Arm Connected? " + armEncoder.isConnected());
    System.out.println("Encoder: " + armEncoder.get());
  }

  public void configArmMotors() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration()); //this reset to default
    armMotor.getConfigurator().apply(Robot.ctreConfigs.armFXConfig); // arm config

    //resetArm(); //dont reset arm in config, let encoder do it
  }

  public void resetArm() { //reset 0
    armMotor.setPosition(0);
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

  public void turnOffArm(){
    armMotor.setControl(new NeutralOut()); 
  }

  public void moveToZeroPosition() {//homing to encdoer set 0 value. not need if using testArmEncoderReset() constantly
    double currentPosition = armEncoder.get();
    double positionError = Constants.ArmConstants.ARM_ZERO_POSITION - currentPosition;
    double falconTarget = positionError;// * Constants.ArmConstants.TICKS_PER_REV;

    armMotor.setControl(armMM.withPosition(falconTarget).withSlot(1));
  }
  public void moveToEncoderPosition(double setpoint) {
    double currentPosition = armEncoder.get();
    double positionError = setpoint - currentPosition;
    double falconTarget = positionError;// * Constants.ArmConstants.TICKS_PER_REV; No need for ticks per rev if sensor to mech ratio

    armMotor.setControl(armMM.withPosition(falconTarget).withSlot(1));
  }

  public boolean isAtZeroPosition() {
  return Math.abs(armEncoder.get() - Constants.ArmConstants.ARM_ZERO_POSITION) < Constants.ArmConstants.ARM_ZERO_POSITION_THRESHOLD;
  } 

  public void testArmEncoderReset() { //constantly reset falcon encoder to REV encoder value
    //In this method of feedback. Set points will be based on REV encoder values.
    double encoderPosition = armEncoder.get();
    armMotor.setPosition(encoderPosition);
  }

 

}
