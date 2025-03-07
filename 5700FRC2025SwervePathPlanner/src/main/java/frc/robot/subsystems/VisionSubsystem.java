// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class VisionSubsystem extends SubsystemBase {
  private final PIDController rotationPID = new PIDController(0.05,0.0001,0); //tx
  private final PIDController forwardPID = new PIDController(2,0.05,0); //ty

  private final PIDController lateralPIDR = new PIDController(2, 0.05, 0);
  private final PIDController lateralPIDL = new PIDController(2, 0.05, 0);

  private final PIDController lateralPID = new PIDController(2, 0.05, 0);

  private static final double DESIRED_FORWARD = 1.0; // desired forward distance
  private static final double DESIRED_LATERAL = 0.0; // desired lateral offset (centered)

  private double forwardCommand;
  private double lateralCommand;
  private double rotationCommand;

  private double lateralCommandL;
  private double lateralCommandR;

  private double[] botPose;

  /** Creates a new Vision. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    boolean targetVisible = LimelightHelpers.getTV("limelight"); // Valid target flag

    botPose = LimelightHelpers.getTargetPose_CameraSpace("limelight"); // Get robot pose in world frame
    
    double currentForward = botPose[2]; // Forward distance
    double currentLateral = botPose[0]; // Lateral offset
    double currentYaw = botPose[4];

    /* 
    if (currentYaw == -180 || currentYaw == 180) {
      currentYaw = 0;
    } else if (currentYaw > 0) {
      currentYaw -= 180;
    } else if (currentYaw < 0) {
      currentYaw += 180;
    }
*/
    //if (currentYaw < 0.25 || currentYaw > -0.25) currentYaw = 0;

    // LEFT: Forward: 0.477; Lateral: 0.164


    if (targetVisible) {
      // Calculate PID commands for forward, lateral, and rotation
      forwardCommand = forwardPID.calculate(currentForward, 0.7); // Target forward distance (1 meter away) 0.477
      lateralCommand = lateralPID.calculate(currentLateral);
      rotationCommand = rotationPID.calculate(currentYaw, 0);

      lateralCommandL = lateralPIDL.calculate(currentLateral, 0.164); // 0.01651
      lateralCommandR = lateralPIDR.calculate(currentLateral, -0.164);
    } else {
      forwardCommand = 0;
      lateralCommand = 0;
      rotationCommand = 0;
    }

    //System.out.println("Current Forward: " + currentForward);
    //System.out.println("Current Lateral: " + currentLateral);
    //System.out.println("Current Yaw: " + currentYaw);
    //System.out.println(forwardCommand);
  }

  public double getForwardCommand() {
    return forwardCommand;
  }

  public double getLateralCommand() {
    return lateralCommand;
  }

  public double getLateralCommandL() {
    return lateralCommandL;
  }

  public double getLateralCommandR() {
    return lateralCommandR;
  }

  public double getRotationCommand() {
    return rotationCommand;
  }

  public boolean targetReached() {
    if (((Math.abs(botPose[2] - 0.05)) < 0.01) && 
        ((Math.abs(botPose[0])) < 0.01) &&
        ((Math.abs(botPose[4])) < 1)) {
      return true;
    } else {
      return false;
    }
  }
}
