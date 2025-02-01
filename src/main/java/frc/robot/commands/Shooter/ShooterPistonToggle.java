// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterPistonToggle extends Command {
  private Shooter shooter;
  private int state;
  /** Creates a new ShooterPistonToggle. */
  public ShooterPistonToggle(Shooter shooter, int state) {
    this.shooter = shooter;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 1){
      shooter.shooterPistonDown();
      System.out.println("Shooter Down");
    }else if(state == -1){
      shooter.shooterPistonUp();
      System.out.println("Shooter Up");
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
