// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.lib.util.MathUtil;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;


public class IntakePistonCMD extends Command {
  private Intake intake;
  private int toggle;
  //private RobotContainer robotContainer;

  /** Creates a new IntakeCMD. */
  public IntakePistonCMD(Intake intake, int toggle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.toggle = toggle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(toggle == 1){
      intake.intakePistonUp();
    }else if(toggle == -1){
      intake.intakePistonDown();
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