// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefaultCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  /** Creates a new ElevatorDefaultCommand. */
  public ElevatorDefaultCommand(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          switch (States.armState) {
      case START:
        elevatorSubsystem.stopElevator();
        break;
      case CLEAR:
        elevatorSubsystem.setElevatorPosition(0);
        break;
      case SCORE:
      elevatorSubsystem.stopElevator();
        break;
      case INTAKE:
      elevatorSubsystem.stopElevator();
      break;
      default:
      elevatorSubsystem.stopElevator();
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
