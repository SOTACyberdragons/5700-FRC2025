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

public class Intake extends SubsystemBase {

  private final DoubleSolenoid intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.intakeOpen, Constants.intakeClose);
  private TalonSRX motor1 = new TalonSRX(Constants.motor1ID);
  
  
  /** Creates a new Intake. */
  public Intake() {
    
  }
  
  public void intakePistonUp(){
    this.intake.set(DoubleSolenoid.Value.kForward);
    
  }
  
  public void intakePistonDown(){
    this.intake.set(DoubleSolenoid.Value.kReverse);
  }

  public void intakeMotorSpeed(double speed){
    this.motor1.set(ControlMode.PercentOutput,speed);
  }
  public void intakePistonToggle(){
    this.intake.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
