// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TalonSRXMotors;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootClose extends Command {
	private final double duration;
  private TalonSRXMotors talonSRXMotors;
	private double endTime;
	private boolean killed = false;
  private Shooter shooter;




  /** Creates a new SetTalonSpeed. */
  public ShootClose(TalonSRXMotors talonSRXMotors,Shooter shooter) {
    this.duration = 0.86;
    this.talonSRXMotors = talonSRXMotors;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talonSRXMotors,shooter);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    killed = false;
    endTime = Timer.getFPGATimestamp() + duration;
    System.out.println("Shooter Close On");
    SmartDashboard.putBoolean("Shooting", true);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterSpeed(1); 
    shooter.shooterPistonUp();
    if(Timer.getFPGATimestamp() > endTime){
      killed = true;
    }

  }
  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return killed;
  }

}
