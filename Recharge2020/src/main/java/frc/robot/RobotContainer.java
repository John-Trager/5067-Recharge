/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;

/** 
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ClimberSubsystem m_climb = new ClimberSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //the drivers and operators controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  //public XboxController m_guitar  = new XboxController(OIConstants.kGuitarPort);



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
            .arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
                         -0.75*m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new POVButton(m_operatorController, 0)
      .whenPressed(() -> m_climb.extendElevatorCAN(0.4))
      .whenReleased(() -> m_climb.stopElevator());

    new POVButton(m_operatorController, 180)
      .whenPressed(() -> m_climb.extendElevatorCAN(-0.5))
      .whenReleased(() -> m_climb.stopElevator());

    new POVButton(m_operatorController, 90)
      .whenPressed(() -> m_climb.startClimb())
      .whenReleased(() -> m_climb.stopClimber());
   
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

   

    return new RunCommand(() -> m_robotDrive.tankDrive(0.4, 0.4), m_robotDrive).withTimeout(2.5);
    //return new InstantCommand();

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> m_robotDrive.stopDriveTrain());
    //return new DriveWithTime(3, 0.2, 0.2);
    //return new RunCommand(() -> m_robotDrive.tankDrive(0, 0), m_robotDrive).withTimeout(2.5).andThen(() -> m_Indexer.runMidIndexer(-0.65)).withTimeout(4.5).andThen(() -> m_robotDrive.tankDrive(0.5, 0.3), m_robotDrive).withTimeout(3);
    //return new RunCommand(() -> m_robotDrive.tankDrive(0, 0), m_robotDrive).withTimeout(2.5);//.andThen(() -> m_Indexer.runMidIndexer(-0.65)).withTimeout(4.5);
    //return new RunCommand(() -> m_robotDrive.tankDrive(0.4, 0.4), m_robotDrive).withTimeout(2.5);
    /* // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return new InstantCommand();
    */
  }
}
