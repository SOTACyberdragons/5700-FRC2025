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


public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor = new TalonFX(Constants.INTAKEConstants.INTAKE_MOTOR_ID, "rio");


  public IntakeSubsystem() {
    configIntakeMotors();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }

  public void configIntakeMotors() {
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration()); //this reset to default
    intakeMotor.getConfigurator().apply(Robot.ctreConfigs.intakeFXConfig); // arm config
   
  }

  
  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void runIntakeCoral() {
    intakeMotor.set(0.5);
  }
  public void runIntakeAlgae() {
    intakeMotor.set(-0.5);
  }

  public void runCoralSlow() {
    intakeMotor.set(0.1);
  }
  public void runAlgaeSlow() {
    intakeMotor.set(-0.1);
  }
  public void runIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public double getIntakeCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void updateIntakeState(){
    if(getIntakeCurrent()>Constants.INTAKEConstants.INTAKE_CURRENT_DIR){
      Constants.intakeState = Constants.IntakeDirection.CORAL;
    }else if(getIntakeCurrent()<-Constants.INTAKEConstants.INTAKE_CURRENT_DIR){
      Constants.intakeState = Constants.IntakeDirection.ALGAE;
    }else{
      Constants.intakeState = Constants.IntakeDirection.NONE;
    }
  }
  public boolean intakeCurrentReached(){
    if(getIntakeCurrent()>Constants.INTAKEConstants.INTAKE_CURRENT_THRESHOLD){
      Constants.intakeState = Constants.IntakeDirection.CORAL;
      return true;
    }else if(getIntakeCurrent()<-Constants.INTAKEConstants.INTAKE_CURRENT_THRESHOLD){
      Constants.intakeState = Constants.IntakeDirection.ALGAE;
      return true;
    }else{
      return false;
    }
    
  }
}