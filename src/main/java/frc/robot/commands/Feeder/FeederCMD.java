// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TalonSRXMotors;

public class FeederCMD extends Command {

  private TalonSRXMotors talonSRXMotors;
  private final int motorOn;
  /** Creates a new FeederCMD. */
  public FeederCMD(TalonSRXMotors talonSRXMotors, int motorOn) {
    this.motorOn = motorOn;
    this.talonSRXMotors = talonSRXMotors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talonSRXMotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("feeder on");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(motorOn == 1){
      System.out.println("feeder Forward");
    } else if(motorOn == -1) {
      System.out.println("feeder Reverse"); 
    }
    talonSRXMotors.setSpeedFeeder(motorOn); 

    
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("feeder off");
    talonSRXMotors.setSpeedFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
