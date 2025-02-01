// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final DoubleSolenoid shooter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.shooterUp, Constants.shooterDown);
  private TalonSRX motor3 = new TalonSRX(Constants.motor3ID); //shooter motor
  private TalonSRX motor4 = new TalonSRX(Constants.motor4ID); //shooter motor


  
  /** Creates a new Intake. */
  public Shooter() {


  }

  public void shooterPistonDown(){
    this.shooter.set(DoubleSolenoid.Value.kForward);
    
  }
  
  public void shooterPistonUp(){
    this.shooter.set(DoubleSolenoid.Value.kReverse);
  }

  public void shooterPistonToggle(){
    this.shooter.toggle();
  }

  public void setSpeed1(double speed) {
    motor3.set(ControlMode.PercentOutput,speed);
    
  }

  public void setSpeed2(double speed) {
    motor4.set(ControlMode.PercentOutput,speed);
  }

  public void setShooterSpeed(double speed){
    motor3.set(ControlMode.PercentOutput,speed);
    motor4.set(ControlMode.PercentOutput,speed);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
