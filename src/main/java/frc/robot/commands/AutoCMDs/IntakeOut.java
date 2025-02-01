// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TalonSRXMotors;



public class IntakeOut extends Command {
  private Intake intake;
  private TalonSRXMotors talonSRXMotors;
	private boolean killed = false;
  //private RobotContainer robotContainer;

  /** Creates a new IntakeCMD. */
  public IntakeOut(Intake intake, TalonSRXMotors talonSRXMotors) {
    this.intake = intake;
    this.talonSRXMotors = talonSRXMotors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, talonSRXMotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    killed = false;
    System.out.println("Intake Down, Full Speed");
    intake.intakePistonDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    talonSRXMotors.setSpeedFeeder(0);
    intake.intakeMotorSpeed(0);
    System.out.println("Intake Up");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }
}
