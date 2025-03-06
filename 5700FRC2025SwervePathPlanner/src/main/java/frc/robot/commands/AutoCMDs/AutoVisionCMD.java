// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDs;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class AutoVisionCMD extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private VisionSubsystem visionsubsystem;
    private int direction;
    private boolean killed;

    private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new VisionMoveToTarget. */
  public AutoVisionCMD(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionsubsystem, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.visionsubsystem = visionsubsystem;
    this.direction = direction;

    addRequirements(drivetrain,visionsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("vision cmd");

    killed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        // Retrieve alignment commands from the vision subsystem
        double forwardCommand = visionsubsystem.getForwardCommand();
        double lateralCommand = visionsubsystem.getLateralCommand();
        double rotationCommand = visionsubsystem.getRotationCommand();

        if (direction == 0) {
          lateralCommand = visionsubsystem.getLateralCommand();
        } else if (direction == 1) {
          lateralCommand = visionsubsystem.getLateralCommandL();
        } else {
          lateralCommand = visionsubsystem.getLateralCommandR();
        }

        // Apply the vision alignment command to the drivetrain
        drivetrain.setControl(
            visionRequest.withVelocityX(forwardCommand)
                         .withVelocityY(-lateralCommand)
                         .withRotationalRate(-rotationCommand)
        );

        if (visionsubsystem.targetReached()) {
          killed = true;
        } else {
          killed = false;
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }
}
