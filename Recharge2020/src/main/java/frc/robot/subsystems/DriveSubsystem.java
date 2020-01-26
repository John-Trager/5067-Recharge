/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  //master Talons for drive
  WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(Constants.DriveConstants.kFrontLeftMotorPort);
  WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(Constants.DriveConstants.kFrontRightMotorPort);
  //slave Talons for drive
  WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(Constants.DriveConstants.kRearLeftMotorPort);
  WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(Constants.DriveConstants.kRearRightMotorPort);
  //differntialDrive for Arcade Drive
  DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry m_odometry;


  /**
  * Creates a new DriveSubsystem.
  */ 
  public DriveSubsystem() {
    //setting motors to factory defualt to prevent unexspected behavior
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    rearLeftMotor.configFactoryDefault();
    rearRightMotor.configFactoryDefault();
    //make rear motor controllers slave to the masters (front controllers)
    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);
    //adjust the direction of the motors
    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(true);
    //have slaves be the same inversion as their corosponding masters
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
   * Drives the robot using tank drive controls.
   *
   * @param left the commanded left-side drivetrain movement
   * @param right the commanded right-side drivetrain movement
   */
  public void tankDrive(double left, double right){
    m_drive.tankDrive(left, right);
  }
 
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading(){
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Zeroes the heading of the robot. (Zeros Yaw axis)
   */
  public void zeroHeading() {
    ahrs.zeroYaw();;
  }

}
