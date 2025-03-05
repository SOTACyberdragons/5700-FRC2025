// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class VisionMoveToTarget extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private VisionSubsystem visionsubsystem;


    private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new VisionMoveToTarget. */
  public VisionMoveToTarget(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.visionsubsystem = visionsubsystem;

    addRequirements(drivetrain,visionsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("vision cmd");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        // Retrieve alignment commands from the vision subsystem
        double forwardCommand = visionsubsystem.getForwardCommand();
        double lateralCommand = visionsubsystem.getLateralCommand();
        double rotationCommand = visionsubsystem.getRotationCommand();

        // Apply the vision alignment command to the drivetrain
        drivetrain.applyRequest(() ->
            visionRequest.withVelocityX(forwardCommand)
                         .withVelocityY(-lateralCommand)
                         .withRotationalRate(-rotationCommand)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
            // Stop the drivetrain when the command ends
            drivetrain.applyRequest(() ->
            visionRequest.withVelocityX(0)
                         .withVelocityY(0)
                         .withRotationalRate(0)
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
