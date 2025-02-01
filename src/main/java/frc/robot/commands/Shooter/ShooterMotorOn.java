// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShooterMotorOn extends Command {

  private Shooter shooter;
  private final Boolean motor;


  /** Creates a new SetTalonSpeed. */
  public ShooterMotorOn(Shooter shooter, Boolean motor) {
    this.motor = motor;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter On");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (motor){
      shooter.setShooterSpeed(1);
      
    }
    else{
      shooter.setShooterSpeed(0);
       
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}