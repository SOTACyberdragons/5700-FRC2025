package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.AutoCMDs.*;
import frc.robot.commands.Feeder.FeederCMD;
import frc.robot.commands.Feeder.FeederDefaultCMD;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /*Intake*/
    private final JoystickButton intakeSmartToggle = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /*Shooter */
    private final int rightTiggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int leftTiggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final JoystickButton ampscore = new JoystickButton(driver, XboxController.Button.kA.value);

    /*Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    private final LED led = new LED();
    private final Shooter shooter = new Shooter();
    private final Swerve s_Swerve = new Swerve();
    private final TalonSRXMotors talonSRXMotors = new TalonSRXMotors();
    private final Intake intake = new Intake();




    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //PP Auto Commands
        NamedCommands.registerCommand("ShootCMD", new ShootFar(talonSRXMotors, shooter));
        NamedCommands.registerCommand("ShootClose", new ShootClose(talonSRXMotors,shooter));
        NamedCommands.registerCommand("FireCMD", new FireCMD(talonSRXMotors,shooter));
        NamedCommands.registerCommand("SmartAutoIntake", new SmartAutoIntake(intake,talonSRXMotors));
        NamedCommands.registerCommand("SmartAutoIntakeC", new SmartAutoIntakeOld(intake,talonSRXMotors));

       

        //Swerve
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> dampen.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
        configureSubsystemDefaults();



        //Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }



    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //turn2angle with A B X Y
        new JoystickButton(driver, XboxController.Button.kY.value).onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.forwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        /* 
        new JoystickButton(driver, XboxController.Button.kA.value).onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.backwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        */
        new JoystickButton(driver, XboxController.Button.kX.value).onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.leftHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        new JoystickButton(driver, XboxController.Button.kB.value).onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.rightHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );

        new POVButton(driver, 0).whileTrue(new FeederCMD(talonSRXMotors,-1) );
        new POVButton(driver, 90).onTrue(new IntakePistonCMD(intake,1));
        new POVButton(driver, 270).onTrue(new IntakePistonCMD(intake,-1));
        new POVButton(driver, 180).whileTrue(new IntakeMotorCMD(intake,-1));

        //Shooting
        new Trigger(() -> driver.getRawAxis(rightTiggerAxis) > .5).whileTrue(new ShooterFarCMD(talonSRXMotors,shooter) );
        new Trigger(() -> driver.getRawAxis(leftTiggerAxis) > .5).whileTrue(new ShooterCloseCMD(talonSRXMotors,shooter) );

        ampscore.whileTrue(new AmpScore(talonSRXMotors, shooter));

        intakeSmartToggle.whileTrue(new SmartIntake(intake,talonSRXMotors));
    
    }

    private void configureSubsystemDefaults() {
        intake.setDefaultCommand(new IntakeDefaultCMD(intake));
        shooter.setDefaultCommand(new ShooterDefaultCMD(shooter));
        talonSRXMotors.setDefaultCommand(new FeederDefaultCMD(talonSRXMotors));

    
    }


    public void setLEDs() {
        if (DriverStation.isDisabled()) {
            led.rainbow();
        } 
        else {
            if (SmartDashboard.getBoolean("Shooting", false)) {
                led.setAllBlink(Color.kGreen, 0.2);
            }
            else if (SmartDashboard.getBoolean("Shot", false)) {
                led.rainbow();
              
            }
            else if (SmartDashboard.getBoolean("Intaking", false)) {
                led.setAllBlink(Color.kRed, 0.2);
            }
            else if(SmartDashboard.getBoolean("Note Got", false)){
                led.setAll(Color.kGreen);
                //System.out.println("Solid Green");
            }
            else if(SmartDashboard.getBoolean("Note Not Set", false)){
                led.setAll(Color.kRed);
            }         
            
            else {
            led.setAll(Color.kCyan);
            //System.out.println("Cyan");
            } 
        }
    }

    /*For AutoBuilder */
     public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
