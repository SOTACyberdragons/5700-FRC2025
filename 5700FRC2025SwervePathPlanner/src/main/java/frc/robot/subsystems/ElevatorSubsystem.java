// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.Constants.ElevatorConstants.ElevatorSelector;

public class ElevatorSubsystem extends SubsystemBase {


  /** Creates a new ElevatorSubsystem. */
  private TalonFX elevatorMasterMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_MASTER_ID, "rio");
  private TalonFX elevatorFollowMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_FOLLOW_ID,"rio");

  private MotionMagicVoltage elevatorMM = new MotionMagicVoltage(0);
  private PositionVoltage elevatorPV = new PositionVoltage(0);

  public ElevatorSubsystem() {
    configElevatorMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //System.out.println("Elevator Position: " + getElevatorPosition());
  }

  public void configElevatorMotors() {
    //elevatorMasterMotor.getConfigurator().apply(new TalonFXConfiguration()); //this reset to default
    elevatorMasterMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorFXConfig); //elevator config

    //elevatorFollowMotor.getConfigurator().apply(new TalonFXConfiguration());
    elevatorFollowMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorFXConfig);

    //resetElevator();
  }

  public void resetElevator() { 
    elevatorMasterMotor.setPosition(0);
    elevatorFollowMotor.setPosition(0);
  }

  public void setElevatorPosition(double position) {
    //double setpoint = levelChoice.getHeight();
    elevatorMM.Position = position;
    elevatorMasterMotor.setControl(elevatorMM.withPosition(position).withSlot(0));
    elevatorFollowMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), true));
  }
  
  public double getElevatorPosition() {
    return elevatorMasterMotor.getPosition().getValueAsDouble();
  }

  public void stopElevator() {
    elevatorMasterMotor.set(0);
    elevatorFollowMotor.set(0);
  }
  public void runElevator() {
    elevatorMasterMotor.set(-0.1);
    elevatorFollowMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), true));
    //elevatorFollowMotor.set(0);
  }

  public void turnOffElevator(){
    elevatorMasterMotor.setControl(new NeutralOut()); 
    elevatorFollowMotor.setControl(new NeutralOut()); 
  }
 

}
