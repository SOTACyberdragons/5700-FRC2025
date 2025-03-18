// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.Constants.ElevatorConstants.ElevatorSelector;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorL4CMD extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private boolean killed;

  /** Creates a new ElevatorCommand. */
  public ElevatorL4CMD(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.


    this.elevatorSubsystem = elevatorSubsystem;
    //this.levelChoice = levelChoice;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    killed = false;

    System.out.println("elevator L4 cmd");
    //elevatorSubsystem.setElevatorPosition(levelChoice);
    elevatorSubsystem.setElevatorPosition(5.79);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //elevatorSubsystem.runElevator();
    if(Math.abs(elevatorSubsystem.getElevatorPosition() - 5.79) < 0.2){
      killed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevatorSubsystem.stopElevator();
    //elevatorSubsystem.setElevatorPosition(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }
}
