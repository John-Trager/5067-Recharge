/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private DriveSubsystem m_drive = new DriveSubsystem();

  private RobotContainer m_robotContainer;

  //timer for timed drivinf
  private static Timer timer = new Timer();

  private boolean timerStarted = false;

  double fwd1, rot1, time1, time2, fwd2, rot2;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    fwd1 = 0.2;
    rot1 = 0;
    fwd2 = 0;
    rot2 = 0.2;
    time1 = 6;
    time2 = 10;

    SmartDashboard.putNumber("fwd1", fwd1);
    SmartDashboard.putNumber("fwd2", fwd2);
    SmartDashboard.putNumber("rot1", rot1);
    SmartDashboard.putNumber("rot2", rot2);
    SmartDashboard.putNumber("time1", time1);
    SmartDashboard.putNumber("time2", time2);


    timer.reset();


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

    double dashFwd1 = SmartDashboard.getNumber("fwd1", 0);
    double dashFwd2 = SmartDashboard.getNumber("fwd2", 0);
    double dashRot1 = SmartDashboard.getNumber("rot1", 0);
    double dashRot2 = SmartDashboard.getNumber("rot2", 0);
    double dashTime1 = SmartDashboard.getNumber("time1", 0);
    double dashTime2 = SmartDashboard.getNumber("time2", 0);

    if((dashFwd1 != fwd1)) {fwd1 = dashFwd1; }
    if((dashFwd2 != fwd2)) {fwd2 = dashFwd2; }
    if((dashRot1 != rot1)) {rot1 = dashRot1; }
    if((dashRot2 != rot2)) {rot2 = dashRot2; }
    if((dashTime1 != time1)) {time1 = dashTime1; }
    if((dashTime2 != time2)) {time2 = dashTime2; }



  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (!timerStarted){
      timer.start();
      timerStarted = !timerStarted;
    }
    if (timer.get() <= time1) {
      m_drive.arcadeDrive(fwd1, rot1);
      return;
    } else if (timer.get() <= time2){
      m_drive.arcadeDrive(fwd2, rot2);
      return;
    } else {
      return;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
