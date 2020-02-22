/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase { 
  // master Talons for drive
  WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(DriveConstants.kFrontLeftMotorPort);
  WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(DriveConstants.kFrontRightMotorPort);
  //slave Talons for drive
  WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(DriveConstants.kRearLeftMotorPort);
  WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(DriveConstants.kRearRightMotorPort);

  DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

  //timer for timed driving
  private static Timer timer = new Timer();

  private boolean timerStarted = false;

  /**
  * Creates a new DriveSubsystem.
  */ 
  public DriveSubsystem() {

    //setting motors to factory defualt to prevent unexspected behavior
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    rearLeftMotor.configFactoryDefault();
    rearRightMotor.configFactoryDefault();
   
    //sets motors to neutral
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftMotor.setNeutralMode(NeutralMode.Brake);
    rearRightMotor.setNeutralMode(NeutralMode.Brake);


    //make rear motor controllers slave to the masters (front controllers)
    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);
    //adjust the direction of the motors
    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(true);
    //have slaves be the same inversion as their corosponding masters
    rearLeftMotor.setInverted(InvertType.FollowMaster);
    rearRightMotor.setInverted(InvertType.FollowMaster);
    //reset timer
    timer.reset();
  }

  @Override
  public void periodic() {
    
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using tank drive controls.
   *
   * @param left the commanded left-side drivetrain movement
   * @param right the commanded right-side drivetrain movement
   */
  public void tankDrive(double left, double right){
    m_drive.tankDrive(left, right);
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void autonDrive(double fwd1, double rot1, double fwd2, double rot2, double firstMoveTime, double SecondMoveTime){
    if (!timerStarted){
      timer.start();
      timerStarted = !timerStarted;
    }
    if (timer.get() <= firstMoveTime) {
      arcadeDrive(fwd1, rot1);
      return;
    } else if (timer.get() <= SecondMoveTime){
      arcadeDrive(fwd2, rot2);
      return;
    } else {
      return;
    }

  }

}
