// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.Constants.ElevatorConstants.ElevatorSelector;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorRun extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  //private final ElevatorSelector levelChoice;
  private double speed;

  /** Creates a new ElevatorCommand. */
  public ElevatorRun(ElevatorSubsystem elevatorSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    //this.levelChoice = levelChoice;
    this.speed = speed;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("elevator cmd");
    //elevatorSubsystem.setElevatorPosition(levelChoice);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //elevatorSubsystem.runElevator();
elevatorSubsystem.runElevator(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopElevator();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
