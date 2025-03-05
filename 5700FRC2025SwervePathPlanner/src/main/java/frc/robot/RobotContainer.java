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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.Constants.ElevatorConstants.ElevatorSelector;
import frc.robot.commands.*;
import frc.robot.commands.AutoCMDs.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        /* Drivetrain buttons */

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * (MaxSpeed * States.elevatorSlowMode.getSpeedFactor())) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * (MaxSpeed* States.elevatorSlowMode.getSpeedFactor())) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * (MaxAngularRate* States.elevatorSlowMode.getSpeedFactor())) // Drive counterclockwise with negative X (left)
            )
        );
        
        //Default Commands
        arm.setDefaultCommand(new ArmDefaultCommand(arm));
        elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator));
        intake.setDefaultCommand(new IntakeDefaultCommand(intake));

        /* 
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);



        /* Elevator */

        //Elevator Height Commands
        joystick.x().whileTrue(
            new ParallelCommandGroup(
                new ArmCommand(arm, -0.05),
                new ElevatorCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT)
            )
        );
        joystick.x().onFalse( //needs testing lifts elevator up a bit after coral score to prevent arm hitting
            new ElevatorClearCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT+0.5)
        );

        joystick.y().whileTrue(
            new ParallelCommandGroup(
                new ArmCommand(arm, -0.07),
                new ElevatorCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT)
            )
        );

        joystick.y().onFalse( //needs testing lifts elevator up a bit after coral score to prevent arm hitting
        new ElevatorClearCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT+1.0)
        );


        joystick.a().whileTrue(
            new ParallelCommandGroup(
                new ArmCommand(arm, -0.04),
                new ElevatorCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT)
            )
        );

        joystick.a().onFalse( //needs testing lifts elevator up a bit after coral score to prevent arm hitting
        new ElevatorClearCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT+0.5)
        );

        //Intake Coral From station
        joystick.rightBumper().whileTrue(
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.37),
                new ElevatorCommand(elevator, 0),
                new IntakeCommand(intake,1)
            )
        );

        //Outtake Coral
        joystick.leftTrigger().whileTrue(new IntakeCommand(intake,0));

        //Algae Low posistion Intake
        joystick.leftBumper().whileTrue(
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.25),
                new ElevatorCommand(elevator, 0),
                new IntakeCommand(intake,0)
            )
        );
        

        //Algae High posistion Intake
        joystick.pov(90).whileTrue(
            new ParallelCommandGroup(
                new ArmIntakeCommand(arm, 0.14),
                new ElevatorCommand(elevator, 1.2),
                new IntakeCommand(intake,0)
            )
        );

        //Algae Outake (need to add elevator a barrage angle)
        /*
        joystick.b().whileTrue( //barrage height needs to be measure L4 temp placeholder
            new ParallelCommandGroup(
                new ArmCommand(arm, -0.04),
                new ElevatorCommand(elevator, Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT)
            )
        );
        */
        joystick.rightTrigger().whileTrue(new IntakeCommand(intake,1)); //add enum for or boolean for direction, algae outtake

        
        /*Vision */
        
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            vision.withVelocityX(visionSubsystem.getForwardCommand()).withVelocityY(-visionSubsystem.getLateralCommand()).withRotationalRate(-visionSubsystem.getRotationCommand()))
        );

        //joystick.pov(270).whileTrue(new VisionMoveToTarget(drivetrain, visionSubsystem)); //test this



        /* Arm */

        //joystick.rightBumper().whileTrue(new ArmCommand(arm, Constants.ArmConstants.ARM_ANGLE_1));
        //joystick.rightTrigger().whileTrue(new ArmRunCommand(arm));


    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
