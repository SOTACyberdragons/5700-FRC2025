// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
// import frc.robot.Constants.ElevatorConstants.ElevatorSelector;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmDefaultCommand extends Command {
  private ArmSubsystem armSubsystem;


  /** Creates a new ElevatorCommand. */
  public ArmDefaultCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("arm cmd");
    //elevatorSubsystem.setElevatorPosition(levelChoice);
    //armSubsystem.setArmPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    if(armSubsystem.isAtZeroPosition()){
      armSubsystem.resetArm();
    } else{
      armSubsystem.moveToZeroPosition();
    }
    */
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevatorSubsystem.stopElevator();
    //armSubsystem.setArmPosition(0);
    //elevatorSubsystem.resetElevator();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}