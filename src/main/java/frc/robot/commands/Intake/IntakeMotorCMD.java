// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;



public class IntakeMotorCMD extends Command {

  private Intake intake;
  private final int motorOn;


  /** Creates a new SetTalonSpeed. */
  public IntakeMotorCMD(Intake intake, int motorOn) {
    this.motorOn = motorOn;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake On");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(motorOn == 1){
      System.out.println("Intake Forward");
    } else if(motorOn == -1) {
      System.out.println("Intake Reverse"); 
    } 

    intake.intakeMotorSpeed(motorOn); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake Off");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
