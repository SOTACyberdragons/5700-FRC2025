// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class VisionSubsystem extends SubsystemBase {
  private final PIDController rotationPID = new PIDController(0.0005,0,0); //tx
  private final PIDController forwardPID = new PIDController(0.3,0,0); //ty
  private final PIDController lateralPID = new PIDController(0.3, 0, 0);

  private static final double DESIRED_FORWARD = 1.0; // desired forward distance
  private static final double DESIRED_LATERAL = 0.0; // desired lateral offset (centered)

  private double forwardCommand;
  private double lateralCommand;
  private double rotationCommand;

  /** Creates a new Vision. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    boolean targetVisible = LimelightHelpers.getTV("limelight"); // Valid target flag

    double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("limelight"); // Get robot pose in world frame
    
    double currentForward = botPose[2]; // Forward distance
    double currnetLateral = botPose[0]; // Lateral offset
    double currentYaw = botPose[4];

    if (currentYaw == -180 || currentYaw == 180) {
      currentYaw = 0;
    } else if (currentYaw > 0) {
      currentYaw -= 180;
    } else if (currentYaw < 0) {
      currentYaw += 180;
    }

    //if (currentYaw < 0.25 || currentYaw > -0.25) currentYaw = 0;

    if (targetVisible) {
      // Calculate PID commands for forward, lateral, and rotation
      forwardCommand = forwardPID.calculate(currentForward, 0.05); // Target forward distance (1 meter away)
      lateralCommand = lateralPID.calculate(currnetLateral);
      rotationCommand = rotationPID.calculate(currentYaw, 0);


    } else {
      forwardCommand = 0;
      lateralCommand = 0;
    }

    System.out.println("Current Forward: " + botPose[2]);
    System.out.println("Current Lateral: " + botPose[0]);
    System.out.println("Current Yaw: " + botPose[4]);
    System.out.println("Rotation Command: " + rotationCommand);
  }

  public double getForwardCommand() {
    return forwardCommand;
  }

  public double getLateralCommand() {
    return lateralCommand;
  }

  public double getRotationCommand() {
    return rotationCommand;
  }
}
