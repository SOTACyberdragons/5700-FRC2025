// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.States.IntakeState;
//import frc.robot.Constants.ElevatorConstants.ElevatorSelector;
import frc.robot.commands.*;
import frc.robot.commands.AutoCMDs.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class RobotContainer {



    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.8); 
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.8); 
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4); 

    /* Setting up bindings for necessary control of the swerve drive platform. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric vision = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Elevator */
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();


    /* Arm */
    public final ArmSubsystem arm = new ArmSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /*Intake */
    public final IntakeSubsystem intake = new IntakeSubsystem();

    /* Vision */
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();


    public RobotContainer() {
        NamedCommands.registerCommand("IntakeCMD", new IntakeCMD(intake));
        NamedCommands.registerCommand("OuttakeCMD", new OuttakeCMD(intake));
        NamedCommands.registerCommand("ArmDefault", new ArmCMD(arm));
        NamedCommands.registerCommand("ArmScore", new ArmL2ScoreCMD(arm));
        NamedCommands.registerCommand("ArmScoreL4", new ArmL4ScoreCMD(arm));
        NamedCommands.registerCommand("ElevatorL2", new ElevatorL2CMD(elevator));
        NamedCommands.registerCommand("ElevatorL4", new ElevatorL4CMD(elevator));
       NamedCommands.registerCommand("AutoVision", new AutoVisionCMD(drivetrain, visionSubsystem));
       NamedCommands.registerCommand("ElevatorDefault", new ElevatorDefault(elevator));
       NamedCommands.registerCommand("ArmLowBall", new ArmLowBallAuto(arm));
       NamedCommands.registerCommand("AutoBallVision", new AutoVisionBall(drivetrain, visionSubsystem));
       NamedCommands.registerCommand("ElevatorBall", new ElevatorHighBall(elevator));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        /* Drivetrain buttons */

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        //x = L2, y = L3, a = L4, b = barge, LB4 (left joystick press) = left vision, RB4 (right joystick press) = right vision, RB = station coral
        //RT =outtake current, LB = Low Ball, LT = High Ball, POV robot centric
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                // drive.withVelocityX(-joystick.getLeftY() * (MaxSpeed * States.elevatorSlowMode.getSpeedFactor())) // Drive forward with negative Y (forward)
                //     .withVelocityY(-joystick.getLeftX() * (MaxSpeed* States.elevatorSlowMode.getSpeedFactor())) // Drive left with negative X (left)
                //     .withRotationalRate(-joystick.getRightX() * (MaxAngularRate* States.elevatorSlowMode.getSpeedFactor())) // Drive counterclockwise with negative X (left)
                drive.withVelocityX(xLimiter.calculate(-joystick.getLeftY())* MaxSpeed * States.elevatorSlowMode.getSpeedFactor()) // Drive forward with negative Y (forward)
                .withVelocityY(yLimiter.calculate(-joystick.getLeftX())*MaxSpeed* States.elevatorSlowMode.getSpeedFactor()) // Drive left with negative X (left)
                .withRotationalRate(rotLimiter.calculate(-joystick.getRightX())*MaxAngularRate* States.elevatorSlowMode.getSpeedFactor()) // Drive counterclockwise with negative X (left)
            )
        );
        
        //Default Commands
        arm.setDefaultCommand(new ArmDefaultCommand(arm));
        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator));
        intake.setDefaultCommand(new IntakeDefaultCommand(intake));

        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);


        /*Drive robot centric */
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );

        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );

        
        /* Elevator */

        //Elevator Height Commands
        joystick.x().whileTrue( //L2
            new ParallelCommandGroup(
                new ArmCommand(arm, 0.02),  
                new ElevatorCommand(elevator, 0.72), //0.73
                new ConditionalCommand(
                    new OuttakeCommand(intake, -0.2), // Run outtake if RT is pressed
                    new InstantCommand(), // Do nothing if RT is not pressed
                    () -> joystick.getRightTriggerAxis() > 0.5
                )
            )
        );
 
        joystick.y().whileTrue( //L3
            new ParallelCommandGroup(
                new ArmCommand(arm, 0.009),
                new ElevatorCommand(elevator, 2.75), //3.13
                new ConditionalCommand(
                    new OuttakeCommand(intake, -0.2), // Run outtake if RT is pressed
                    new InstantCommand(), // Do nothing if RT is not pressed
                    () -> joystick.getRightTriggerAxis() > 0.5
                )
            )
        );

        joystick.a().whileTrue( //L4
            new ParallelCommandGroup(
                new ArmCommand(arm, -0.04),
                new ElevatorCommand(elevator, 5.79), //6.55
                new ConditionalCommand(
                    new OuttakeCommand(intake, -0.2), // Run outtake if RT is pressed
                    new InstantCommand(), // Do nothing if RT is not pressed
                    () -> joystick.getRightTriggerAxis() > 0.5
                )
            )
        );

        joystick.b().whileTrue( //barrage height needs to be measure L4 temp placeholder
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, 7.7),
                new ArmCommand(arm, -0.15),
                new ConditionalCommand(
                    new OuttakeCommand(intake, 0.5), // Run outtake if RT is pressed
                    new InstantCommand(), // Do nothing if RT is not pressed
                    () -> joystick.getRightTriggerAxis() > 0.5
                )
            )  
        );

        //Intake Coral From station
        joystick.rightBumper().whileTrue(
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.41), //37
                new ElevatorCommand(elevator, 0),
                new IntakeCommand(intake,1)
            )
        );

        //Algae Low posistion Intake
          joystick.leftBumper().whileTrue(
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.2),
                new ElevatorCommand(elevator, 0),
                new IntakeCommand(intake,0),
                new VisionAlgae(
                    drivetrain, 
                    visionSubsystem,
                    () ->joystick.getRightY(),//forward on right stick
                    () ->-joystick.getLeftX(), //no used
                    () ->-joystick.getRightX() //no used
                )
            )   
        );
    
        //Algae High posistion Intake
        joystick.leftTrigger().whileTrue(
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.14),
                new ElevatorCommand(elevator, 1.2),
                new IntakeCommand(intake,0),
                new VisionAlgae(
                    drivetrain, 
                    visionSubsystem,
                    () ->joystick.getRightY(),//forward on right stick
                    () ->-joystick.getLeftX(), //no used
                    () ->-joystick.getRightX() //no used
                )
            )
        );

        //Processor
        joystick.start().onTrue(         
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.41),
                new ElevatorCommand(elevator, 0),
                new ConditionalCommand(
                    new OuttakeCommand(intake, 0.25), // Run outtake if RT is pressed
                    new InstantCommand(), // Do nothing if RT is not pressed
                    () -> joystick.getRightTriggerAxis() > 0.5
                )
            )
        );

        //Outtake All
        //joystick.rightTrigger().whileTrue(new OuttakeCommand(intake,0)); // ball outtake 1 is run coral
        
        /*Vision Buttons */
        joystick.rightStick().whileTrue(new VisionMoveToTargetRight(
            drivetrain, 
            visionSubsystem,
            () ->joystick.getLeftY(),
            () ->-joystick.getLeftX(),
            () ->-joystick.getRightX())
        ); 
        joystick.leftStick().whileTrue(new VisionMoveToTargetLeft(
            drivetrain, 
            visionSubsystem,
            () ->-joystick.getLeftY(),
            () ->-joystick.getLeftX(),
            () ->-joystick.getRightX())

        ); 

        /*Old up command */
        // joystick.x().onFalse( //needs testing lifts elevator up a bit after coral score to prevent arm hitting
        // new ParallelCommandGroup(
        //     //new ArmHoldUpCommand(arm,-0.05),
        //     new ElevatorClearCommand(elevator, 0.73+0.5)
        // )
        // );

        // joystick.y().onFalse( //needs testing lifts elevator up a bit after coral score to prevent arm hitting
        //     new ElevatorClearCommand(elevator,3.13+2.0)
        // );

        // joystick.a().onFalse( //needs testing lifts elevator up a bit after coral score to prevent arm hitting
        // new ElevatorClearCommand(elevator, 7.1+0.5)
        // );

        /*Manual Commands */
        // joystick.pov(270).whileTrue(new ElevatorRun(elevator, -0.1)
        // );

        // joystick.pov(90).whileTrue(new ElevatorRun(elevator, 0.1)
        // );
        // joystick.pov(0).whileTrue(new ArmRunCommand(arm, 0.05)
        // );
        // joystick.pov(180).whileTrue(new ArmRunCommand(arm, -0.05)
        // );


        /* 
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        // joystick.start().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(joystick.getLeftY()*(MaxSpeed * 0.05))
        //     .withVelocityY(joystick.getLeftX()*(MaxSpeed * 0.05)))
        // );

        //Outtake Coral
        //joystick.pov(0).whileTrue(new OuttakeCommand(intake,0)); //0 is runalgae so outtake
        //joystick.rightStick().whileTrue(new OuttakeCommand(intake,0)); 

      
        //Algae Outake (need to add elevator a barrage angle)
        

        //processor
        // joystick.pov(0).whileTrue(
        //     new ParallelCommandGroup(
        //         new ArmIntakeCommand(arm, 0.46),
        //         new ElevatorCommand(elevator, 0)
        //     )
        // );

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
