/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //the drivers and operators controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive
            .arcadeDrive(m_driverController.getY(GenericHID.Hand.kRight),
                         -m_driverController.getX(GenericHID.Hand.kLeft)), m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
      
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.25))
        .whenReleased(() -> m_robotDrive.setMaxOutput(0.5));

    // Rotates to Vison Target
    new JoystickButton(m_driverController, Button.kX.value)
        .toggleWhenPressed(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)));

    //turns limelight leds off
    new JoystickButton(m_driverController, Button.kB.value)
     .whenPressed(() -> Limelight.setLedMode(frc.robot.utils.Limelight.LightMode.eOff));

    //turns limelight leds on
    new JoystickButton(m_driverController, Button.kA.value)
     .whenPressed(() -> Limelight.setLedMode(LightMode.eOn));

    //turns the shooter on
    new JoystickButton(m_operatorController, Button.kA.value)
     .whileHeld(() -> m_shooter.startShooter(m_operatorController.getTriggerAxis(GenericHID.Hand.kRight)));

    //rotates to set angle from D-PAD
    new POVButton(m_driverController, 90)
      .whenPressed(new TurnToAngle(m_robotDrive, 90, m_driverController.getY(GenericHID.Hand.kRight)));
    
    /**new POVButton(m_driverController, 45)
      .whenPressed(new TurnToAngle(m_robotDrive, 45, m_driverController.getY(GenericHID.Hand.kRight)));

      new POVButton(m_driverController, 90)
      .whenPressed(new TurnToAngle(m_robotDrive, 90, m_driverController.getY(GenericHID.Hand.kRight)));

    new POVButton(m_driverController, 135)
      .whenPressed(new TurnToAngle(m_robotDrive, 135, m_driverController.getY(GenericHID.Hand.kRight)));
      
    new POVButton(m_driverController, 180)
      .whenPressed(new TurnToAngle(m_robotDrive, 180, m_driverController.getY(GenericHID.Hand.kRight)));

    new POVButton(m_driverController, 225)
      .whenPressed(new TurnToAngle(m_robotDrive, 225, m_driverController.getY(GenericHID.Hand.kRight)));

    new POVButton(m_driverController, 270)
      .whenPressed(new TurnToAngle(m_robotDrive, 270, m_driverController.getY(GenericHID.Hand.kRight)));

    new POVButton(m_driverController, 315)
      .whenPressed(new TurnToAngle(m_robotDrive, 315, m_driverController.getY(GenericHID.Hand.kRight)));    
    */
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return new InstantCommand();
  }
}
