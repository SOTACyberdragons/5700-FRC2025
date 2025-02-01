// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;



public class IntakeDefaultCMD extends Command {
  private Intake intake;


  /** Creates a new IntakeCMD. */
  public IntakeDefaultCMD(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //intake.intakePistonDown();
    
    System.out.println("Intake Default");
    
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    intake.intakeMotorSpeed(0);
    
  }

}