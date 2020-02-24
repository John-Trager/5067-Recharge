/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.TurnToAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.swagGuitar;

import static edu.wpi.first.wpilibj.XboxController.Button;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

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
  private ClimberSubsystem m_Climb = new ClimberSubsystem();
  private BallIntakeSubsystem m_BallIntake = new BallIntakeSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //the drivers and operators controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  public XboxController m_guitar  = new XboxController(OIConstants.kGuitarPort);



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

    /**
     * Drive Controls
     * 
     *  - Right Bumper: Slow Drive
     *  - TODO: X button: Rotate then shoot balls (just rotates rn)
     */

    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whenPressed(() -> m_robotDrive.setMaxOutput(0.25))
      .whenReleased(() -> m_robotDrive.setMaxOutput(0.8));

    // Rotates to Vison Target, spins up motors based on distance, when released stops motors
    new JoystickButton(m_driverController, Button.kX.value)
      .whenHeld(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)))
      .whenPressed(() -> m_shooter.distanceVelocityShooter())
      .whenReleased(() -> m_shooter.stopShooterMotors());
      //.toggleWhenPressed(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)));


    //turns the shooter on
    new JoystickButton(m_operatorController, Button.kY.value)
      .whileHeld(() -> m_shooter.startShooter(m_operatorController.getTriggerAxis(GenericHID.Hand.kRight)));



    /**
     * Op - Controls
     *  
     *  - Y button: extend elevator to max set height
     */
    
    //starts the climber winch
    new JoystickButton(m_operatorController, Button.kB.value)
        .whileHeld(() -> m_Climb.startClimb(m_operatorController.getTriggerAxis(Hand.kLeft)));

    //extends the elevator 
    new JoystickButton(m_operatorController, Button.kB.value)
        .whileHeld(() -> m_Climb.extendElevatorCAN(m_operatorController.getTriggerAxis(Hand.kRight)));

    
    // Guitar code

    //extends the elevator 
    new JoystickButton(m_guitar, swagGuitar.plusButton)
        .whileHeld(() -> m_Climb.extendElevatorPID());

    //retracts the elevator 
    new JoystickButton(m_guitar, swagGuitar.minusButton)
        .whileHeld(() -> m_Climb.retractElevatorCAN());

    //extends the intake and runs intake motor stops motor when released
    new JoystickButton(m_guitar, swagGuitar.greenButton)
        .whenPressed(() -> m_BallIntake.extendIntake())
        .whileHeld(() -> m_BallIntake.intakeSetSpeed(0.3))
        .whenReleased(() -> m_BallIntake.stopIntakeMotor());

    //retracts the intake and stops the motor
    new JoystickButton(m_guitar, swagGuitar.greenButton)
        .whenPressed(() -> m_BallIntake.retractIntake())
        .whenPressed(() -> m_BallIntake.stopIntakeMotor());
       
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(AutoConstants.ksVolts,
                                       AutoConstants.kvVoltSecondsPerMeter,
                                       AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics,
            10);
            
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    String trajectoryJSON = "path1.wpilib.json";
    Trajectory trajectory = null;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

     // An example trajectory to follow.  All units in meters.
     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1, 0, new Rotation2d(0)),
      // Pass config
      config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts,
                                   AutoConstants.kvVoltSecondsPerMeter,
                                   AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
   
   /* // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return new InstantCommand();
    */
  }
}
