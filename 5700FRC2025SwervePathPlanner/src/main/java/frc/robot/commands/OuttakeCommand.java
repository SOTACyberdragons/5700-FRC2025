// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ElevatorState;
import frc.robot.States.IntakeState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private double speed;

  private boolean killed;

  /** Creates a new IntakeCommand. Outtake Coral is - , Outtake Ball is +, 1 is max*/
  public OuttakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.speed = speed;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    killed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // switch (States.intakeState) {
    //   case CORAL:
    //     intakeSubsystem.runIntakeCoral();
    //     break;
    //   case ALGAE:
    //     //intakeSubsystem.runIntakeAlgae();
    //     intakeSubsystem.setIntakeVoltage(1);
    //     break;
    //   default:
    //     intakeSubsystem.stopIntake();
    //     break;
    // }
    //killed = intakeSubsystem.intakeCurrentReached();
    
    // if(direction == 1){
    // intakeSubsystem.runIntakeCoral();
    // States.intakeState = IntakeState.NEUTRAL;
    
    // }else {
    //   intakeSubsystem.runIntakeAlgae();
    //   States.intakeState = IntakeState.NEUTRAL;
    // }
    intakeSubsystem.runIntakeMotor(speed);
    States.intakeState = IntakeState.NEUTRAL;
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
