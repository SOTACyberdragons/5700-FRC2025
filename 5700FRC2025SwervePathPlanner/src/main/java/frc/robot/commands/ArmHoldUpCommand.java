// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
// import frc.robot.Constants.ElevatorConstants.ElevatorSelector;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmHoldUpCommand extends Command {
  private ArmSubsystem armSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private double armAngle;
  private double delay;
  private boolean killed;

  /** Creates a new ElevatorCommand. */
  public ArmHoldUpCommand(ArmSubsystem armSubsystem, double armAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.armSubsystem = armSubsystem;
    //this.levelChoice = levelChoice;
    this.armAngle = armAngle;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("arm cmd");
    killed = false;
    //elevatorSubsystem.setElevatorPosition(levelChoice);
    //armSubsystem.testArmEncoderReset();
    delay  = Timer.getFPGATimestamp() + 0.5;
    if(armSubsystem.getArmPosition()>0.1){
      armSubsystem.testArmEncoderReset();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //elevatorSubsystem.runElevator();
    armSubsystem.setArmPosition(armAngle);
    if(Timer.getFPGATimestamp() > delay){
      killed = true;
    }

  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevatorSubsystem.stopElevator();
    //armSubsystem.setArmPosition(0.42);//default starting
    //elevatorSubsystem.resetElevator();

    //armSubsystem.setArmPosition(0.14);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }
}
