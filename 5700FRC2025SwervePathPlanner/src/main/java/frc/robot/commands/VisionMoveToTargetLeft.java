// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class VisionMoveToTargetLeft extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private VisionSubsystem visionsubsystem;

    private final PIDController rotationPID = new PIDController(0.05,0,0); //tx
    private final PIDController forwardPID = new PIDController(2.5,0,0); //ty
    private final PIDController lateralPID = new PIDController(2, 0, 0.01);

    private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private int direction;

    private double forwardCommand;
    private double lateralCommand;
    private double rotationCommand;

    private Supplier<Double> forwardSupplier;
    private Supplier<Double> lateralSupplier;
    private Supplier<Double> rotationSupplier;

  /** Creates a new VisionMoveToTarget. */
  public VisionMoveToTargetLeft(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionsubsystem,
    Supplier<Double> forwardSupplier, 
    Supplier<Double> lateralSupplier, 
    Supplier<Double> rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.forwardSupplier = forwardSupplier;
    this.lateralSupplier = lateralSupplier;
    this.rotationSupplier = rotationSupplier;
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

    if(visionsubsystem.getTargetVisibleLL1() || visionsubsystem.getTargetVisibleLL2()){
      if(visionsubsystem.getTA2()>=visionsubsystem.getTA1()){

      forwardCommand = forwardPID.calculate(visionsubsystem.getForward2(), 0.32); // Target forward distance (1 meter away) 0.477
      lateralCommand = lateralPID.calculate(visionsubsystem.getLateral2(),-0.28);
      rotationCommand = rotationPID.calculate(visionsubsystem.getRotation2(), 0);

      }else if(visionsubsystem.getTA2()<visionsubsystem.getTA1()){

        forwardCommand = forwardPID.calculate(visionsubsystem.getForward(), 0.32); // Target forward distance (1 meter away) 0.477
        lateralCommand = lateralPID.calculate(visionsubsystem.getLateral(),-0.02);
        rotationCommand = rotationPID.calculate(visionsubsystem.getRotation(), 0);

      }else{
        forwardCommand = 0;//forwardSupplier.get();
        lateralCommand = 0;//lateralSupplier.get();
        rotationCommand = 0;//rotationSupplier.get();
      }
    }else{
      forwardCommand = 0;//forwardSupplier.get();
      lateralCommand = 0;//lateralSupplier.get();
      rotationCommand = 0;//rotationSupplier.get();
    }
    if(Math.abs(visionsubsystem.getRotation2())<0.5 ||Math.abs(visionsubsystem.getRotation())<0.5 ){
      rotationCommand = 0;
    }
      drivetrain.setControl(
      visionRequest.withVelocityX(forwardCommand)//forwardCommand
              .withVelocityY(-lateralCommand)//-lateralCommand
              .withRotationalRate(rotationCommand*0.5)//rotationCommand
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        //     // Stop the drivetrain when the command ends
        // drivetrain.setControl(
        //     visionRequest.withVelocityX(0)
        //                  .withVelocityY(0)
        //                  .withRotationalRate(0)
        // );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
