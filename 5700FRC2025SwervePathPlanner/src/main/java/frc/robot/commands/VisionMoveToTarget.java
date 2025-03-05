// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class VisionMoveToTarget extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController rotationPID = new PIDController(0.01,0,0); //tx
  private final PIDController forwardPID = new PIDController(0.1,0,0); //ty
  private final PIDController lateralPID = new PIDController(0.05, 0, 0);


  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Open-loop control

    // Desired positions (in meters) relative to the tag.
  // For example, if you want to stop 1 meter away from the tag and be perfectly centered laterally:
  private static final double DESIRED_FORWARD = 1.0; // desired forward distance
  private static final double DESIRED_LATERAL = 0.0;   // desired lateral offset (centered)


  /** Creates a new VisionMoveToTarget. */
  public VisionMoveToTarget(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean targetVisible = LimelightHelpers.getTV(""); // Valid target flag

    double[] botPose = LimelightHelpers.getBotPose("");


  // If no target is detected, stop the drivetrain.
  if (!targetVisible ){//|| botPose.length < 6) {
    drivetrain.applyRequest(() -> 
      drive.withVelocityX(0.0)  // No forward movement
           .withVelocityY(0.0)  // No lateral movement
           .withRotationalRate(0.0)  // No rotation
    );
    return;
  }

 
    double currentForward = botPose[0];  // Forward distance
    double currentLateral = botPose[1];    // Lateral offset
    double currentYaw = botPose[5];      

    // Compute PID outputs for forward and lateral corrections
    double forwardCommand = forwardPID.calculate(currentForward, DESIRED_FORWARD);
    double lateralCommand = lateralPID.calculate(currentLateral, DESIRED_LATERAL);
    double rotationCommand = rotationPID.calculate(currentYaw, 0);
    // Apply the drive request to the drivetrain with calculated speeds
    drivetrain.applyRequest(() -> 
      drive.withVelocityX(forwardCommand*0.2)  // Forward movement
           .withVelocityY(lateralCommand*0.2)  // Lateral movement
           .withRotationalRate(rotationCommand*0.2)  // Rotation to align with target
    );
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
