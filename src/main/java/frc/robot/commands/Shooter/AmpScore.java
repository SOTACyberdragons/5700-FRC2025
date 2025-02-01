// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonSRXMotors;
import frc.robot.subsystems.Shooter;


public class AmpScore extends Command {

  private TalonSRXMotors talonSRXMotors;
  private Shooter shooter;




  /** Creates a new SetTalonSpeed. */
  public AmpScore(TalonSRXMotors talonSRXMotors,Shooter shooter) {
    this.talonSRXMotors = talonSRXMotors;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talonSRXMotors,shooter);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AMP");
    //shooter.shooterPistonUp();
    SmartDashboard.putBoolean("Shooting", true);
    //feederdelay = Timer.getFPGATimestamp() + 0.2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.shooterPistonDown();
    shooter.setShooterSpeed(1);
    talonSRXMotors.setSpeedFeeder(1);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AMP Release");
    shooter.setShooterSpeed(0);
    talonSRXMotors.setSpeedFeeder(0);
    SmartDashboard.putBoolean("Shooting", false);
    SmartDashboard.putBoolean("Shot", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}

