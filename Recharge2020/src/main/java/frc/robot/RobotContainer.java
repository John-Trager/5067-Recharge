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
import frc.robot.subsystems.BallIndexSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;
import frc.robot.commands.DriveWithTime;
import frc.robot.commands.TurnToTarget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private BallIntakeSubsystem m_BallIntake = new BallIntakeSubsystem();
  private BallIndexSubsystem m_Indexer = new BallIndexSubsystem();
  private ClimberSubsystem m_climb = new ClimberSubsystem();

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

    /**
     * Drive Controls
     * 
     *  Left Bumper: Slow Drive
     *  TODO: X button: Rotate and shoot
     *  Right Bumper: BallIntake
     *  Y -ball intake retract
     */

    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kBumperLeft.value)
      .whenPressed(() -> m_robotDrive.setMaxOutput(0.3))
      .whenReleased(() -> m_robotDrive.setMaxOutput(1));

   // new JoystickButton(m_driverController, Button.kBumperRight.value)
     // .whenPressed(() -> m_BallIntake.extendIntake())
     // .whileHeld(() -> m_BallIntake.intakeSetSpeed(0.4))
     // .whenReleased(() -> m_BallIntake.stopIntakeMotor());
  
    //retracts the intake and stops the motor
    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_BallIntake.extendIntake())
      .whileHeld(() -> m_BallIntake.intakeSetSpeed(0.5))
      .whenReleased(() -> m_BallIntake.stopIntakeMotor());

    new JoystickButton(m_driverController, Button.kB.value)
      .whenPressed(() -> Limelight.setLedMode(LightMode.eOn))
      .whenPressed(() -> m_shooter.startShooter(0.7))
      .whenReleased(() -> m_shooter.stopShooterMotors());

    new JoystickButton(m_driverController, Button.kA.value)
      .whenPressed(() -> m_BallIntake.retractIntake())
      .whenPressed(() -> m_BallIntake.stopIntakeMotor());

    new JoystickButton(m_driverController, Button.kStart.value)
      .whenPressed(() -> m_Indexer.runMidIndexer(0.65))
      .whileHeld(() -> m_Indexer.runBackIndexer(0.2))
      .whenReleased(() -> m_Indexer.stopIndexer());

    new JoystickButton(m_driverController, Button.kBack.value)
      .whenPressed(() -> m_Indexer.runMidIndexer(-0.65))
      .whileHeld(() -> m_Indexer.runBackIndexer(-0.2))
      .whenReleased(() -> m_Indexer.stopIndexer())
      .whileHeld(() -> m_BallIntake.intakeSetSpeed(-0.5))
      .whenReleased(() -> m_BallIntake.stopIntakeMotor());

    new POVButton(m_driverController, 0)
      .whenPressed(() -> m_climb.extendElevatorCAN(0.4))
      .whenReleased(() -> m_climb.stopElevator());

    new POVButton(m_driverController, 180)
      .whenPressed(() -> m_climb.extendElevatorCAN(-0.6))
      .whenReleased(() -> m_climb.stopElevator());

    new POVButton(m_driverController, 90)
      .whenPressed(() -> m_climb.startClimb())
      .whenReleased(() -> m_climb.stopClimber());



    // Turns LL light on, Rotates to Vison Target, spins up motors based on distance, sends balls to shooter, when released stops motors & turns off limelight
    new JoystickButton(m_driverController, Button.kX.value)
      .whenPressed(() -> Limelight.setLedMode(LightMode.eOn))
      .whenHeld(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)))
      .whenPressed(() -> m_shooter.startShooter())
      .whenReleased(() -> m_shooter.stopShooterMotors());
      //.whenReleased(() -> Limelight.setLedMode(LightMode.eOff));
      //.toggleWhenPressed(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)));


    //turns the shooter on
    //new JoystickButton(m_operatorController, Button.kY.value)
      //.whileHeld(() -> m_shooter.startShooter(m_operatorController.getTriggerAxis(GenericHID.Hand.kRight)));



    /**
     * Op - Controls
     *  
     *  - Y button: extend elevator to max set height
     */

    //extends the intake and runs intake motor stops motor when released
    new JoystickButton(m_operatorController, Button.kY.value)
        .whenPressed(() -> m_BallIntake.extendIntake())
        .whileHeld(() -> m_BallIntake.intakeSetSpeed(0.5))
        .whenReleased(() -> m_BallIntake.stopIntakeMotor());

    //retracts the intake and stops the motor
    new JoystickButton(m_operatorController, Button.kA.value)
        .whenPressed(() -> m_BallIntake.retractIntake())
        .whenPressed(() -> m_BallIntake.stopIntakeMotor());

    //TEST running the indexer
    new JoystickButton(m_operatorController, Button.kX.value)
        .whenPressed(() -> m_Indexer.runMidIndexer(0.5))
        .whenPressed(() -> m_Indexer.runBackIndexer(0.5))
        .whenReleased(() -> m_Indexer.stopIndexer());
    
    //TEST running the indexer
    new JoystickButton(m_operatorController, Button.kB.value)
        .whenPressed(() -> m_Indexer.runMidIndexer(0.5))
       //.whenPressed(() -> m_Indexer.runBackIndexer(-0.5))
        .whenReleased(() -> m_Indexer.stopIndexer());

    //moves the shooter index down 
    new JoystickButton(m_operatorController, Button.kBack.value)
        .whenPressed(() -> m_Indexer.runMidIndexer(-0.65))
        .whileHeld(() -> m_Indexer.runBackIndexer(-0.2))
        .whenReleased(() -> m_Indexer.stopIndexer())
        .whileHeld(() -> m_BallIntake.intakeSetSpeed(-0.5))
        .whenReleased(() -> m_BallIntake.stopIntakeMotor());

    //Extend elevator PID
    /*
    new JoystickButton(m_operatorController, Button.kBumperRight.value)
        .whenPressed(() -> m_climb.extendElevatorPID());
    */
    
    new JoystickButton(m_operatorController, Button.kBumperRight.value)
        .whenPressed(() -> m_climb.elevatorPower(0.4))
        .whenReleased(() -> m_climb.stopElevator());
    
    //makes elevator retract bang-bang loop
    new JoystickButton(m_operatorController, Button.kBumperLeft.value)
        .whenPressed(() -> m_climb.elevatorPower(-0.6))
        .whenReleased(() -> m_climb.stopElevator());

    new JoystickButton(m_operatorController, Button.kStart.value)
        .whenPressed(() -> m_climb.startClimb())
        .whenReleased(() -> m_climb.stopClimber());
    
    
    
    /**
     * Op - Controls BUT for guitar
     *  
     *  - Y button: extend elevator to max set height
     */    
    
     //extends the intake and runs intake motor stops motor when released
    new POVButton(m_guitar, 180)
      .whenPressed(() -> m_BallIntake.extendIntake())
      .whileHeld(() -> m_BallIntake.intakeSetSpeed(0.4))
      .whenReleased(() -> m_BallIntake.stopIntakeMotor());

    //retracts the intake and stops the motor
    new POVButton(m_guitar, 0)
      .whenPressed(() -> m_BallIntake.retractIntake())
      .whenPressed(() -> m_BallIntake.stopIntakeMotor());

    //TEST running the indexer
    new JoystickButton(m_guitar, swagGuitar.plusButton)
        .whenPressed(() -> m_Indexer.runMidIndexer(0.5))
        .whenPressed(() -> m_Indexer.runBackIndexer(0.5))
        .whenReleased(() -> m_Indexer.stopIndexer());

    //TEST running the indexer
    new JoystickButton(m_guitar, swagGuitar.minusButton)
        .whenPressed(() -> m_Indexer.runMidIndexer(-0.5))
        .whenPressed(() -> m_Indexer.runBackIndexer(-0.5))
        .whenReleased(() -> m_Indexer.stopIndexer());

    //moves the shooter index down 
    new JoystickButton(m_guitar, swagGuitar.swagBar)
        .whileHeld(() -> m_Indexer.runBackIndexer(-0.2))
        .whenReleased(() -> m_Indexer.stopIndexer());

    //Extend elevator PID
    new JoystickButton(m_guitar, swagGuitar.greenButton)
        .whenPressed(() -> m_climb.extendElevatorPID());

    //makes elevator retract bang-bang loop
    new JoystickButton(m_guitar, swagGuitar.redButton)
        .whenPressed(() -> m_climb.retractElevatorCAN())
        //.whenPressed(() -> m_climb.startClimb(m_guitar.getRawAxis(swagGuitar.yAxis)))
        .whenReleased(() -> m_climb.stopElevator());

    // Turns LL light on, Rotates to Vison Target, spins up motors based on distance, sends balls to shooter, when released stops motors & turns off limelight
    new JoystickButton(m_guitar, swagGuitar.blueButton)
      .whenPressed(() -> Limelight.setLedMode(LightMode.eOn))
      .whenHeld(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)))
      //.whenPressed(() -> m_shooter.distanceVelocityShooter())
      //.whileHeld(() -> m_Indexer.ballsToShooter())
      //.whenReleased(() -> m_shooter.stopShooterMotors())
      .whenReleased(() -> Limelight.setLedMode(LightMode.eOff));
      ///.toggleWhenPressed(new TurnToTarget(m_robotDrive, m_driverController.getY(GenericHID.Hand.kRight)));
          
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
