/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  //master Talons for drive
  WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(Constants.DriveConstants.kFrontLeftMotorPort);
  WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.DriveConstants.kFrontRightMotorPort);
  //slave Talons for drive
  WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(Constants.DriveConstants.kRearLeftMotorPort);
  WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(Constants.DriveConstants.kRearRightMotorPort);
  //differntialDrive for Arcade Drive
  DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);


  //constructer method
  public DriveSubsystem() {
    //setting motors to factory defualt to prevent unecpected behavior
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    rearLeftMotor.configFactoryDefault();
    rearRightMotor.configFactoryDefault();
    //make talons slave to the masters
    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);
    //adjust the direction of the motors
    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(false);
     
    rearLeftMotor.setInverted(InvertType.FollowMaster);
    rearRightMotor.setInverted(InvertType.FollowMaster);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
