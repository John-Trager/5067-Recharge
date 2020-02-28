/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithTime extends CommandBase {
  //TODO: finish command and figure out how to add Drivesubsystem

  DriveSubsystem m_drive = new DriveSubsystem();
  // Initialization of the variables which will allow imported variables to be public(making usable throughout the DriveWithTime command).
  private double seconds = 0;
  private double leftSpeeds= 0;
  private double rightSpeeds = 0;

  //Initializes a timer
  private Timer timer = new Timer();

  /**
 * This command will take in speeds (left and right) and time, for which is will set the motors to the x speed for y seconds.
 * 
 * @param seconds        amount of seconds the robot motors will be activated for  
 * @param leftSpeeds     when the motor is activated, this is the speed at which the LEFT motors will be. speed can be inbetween -1 to 1.
 * @param rightSpeeds    when the motor is activated, this is the speed at which the RIGHT motors will be. speed can be inbetween -1 to 1.
 */
  public DriveWithTime(double seconds, double leftSpeed, double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Sets the variables to public to allow their usage throughout the DriveWithTime command.
    this.seconds = seconds;
    this.leftSpeeds = leftSpeed;
    this.rightSpeeds = rightSpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Timer is started and will start counting UP from 0
    timer.start();
    //m_drive.setSafetyDisabledForAll();
    //addRequirements(DriveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(leftSpeeds,rightSpeeds);
    m_drive.m_drive.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDriveTrain();
    //m_drive.setSafetyEnabledForAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    *Will return true when the timer's count reaches the inputed seconds, this will stop the command, disabling the motors.
    *when returning false, the command will continue to run, and the motors will continue to spin at their inputed rate.
    */
    return timer.get() >= seconds;
  }
}
