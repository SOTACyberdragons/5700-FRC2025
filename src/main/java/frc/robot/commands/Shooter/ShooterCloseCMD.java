// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonSRXMotors;
import frc.robot.subsystems.Shooter;


public class ShooterCloseCMD extends Command {

  private double feederdelay;
  private TalonSRXMotors talonSRXMotors;
  private Shooter shooter;
  private boolean killed = false;




  /** Creates a new SetTalonSpeed. */
  public ShooterCloseCMD(TalonSRXMotors talonSRXMotors,Shooter shooter) {
    
    this.talonSRXMotors = talonSRXMotors;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talonSRXMotors,shooter);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    killed = false;
    System.out.println("Shooting Speaker");
    SmartDashboard.putBoolean("Shooting", true);
    feederdelay = Timer.getFPGATimestamp() + 0.85;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shooterPistonUp();
    shooter.setShooterSpeed(1);
    if(Timer.getFPGATimestamp() > feederdelay){
    talonSRXMotors.setSpeedFeeder(1);
    }
    if(!talonSRXMotors.getFeederBeamBreak()){
      SmartDashboard.putBoolean("Shot", true);
      SmartDashboard.putBoolean("Shooting", false);
    }

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooting Release");
    SmartDashboard.putBoolean("Shot", false);
    SmartDashboard.putBoolean("Shooting", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }


}
