/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // master Talons for drive
  WPI_TalonFX frontLeftMotor = new WPI_TalonFX(DriveConstants.kFrontLeftMotorPort);
  WPI_TalonFX frontRightMotor = new WPI_TalonFX(DriveConstants.kFrontRightMotorPort);
  //slave Talons for drive
  WPI_TalonFX rearLeftMotor = new WPI_TalonFX(DriveConstants.kRearLeftMotorPort);
  WPI_TalonFX rearRightMotor = new WPI_TalonFX(DriveConstants.kRearRightMotorPort);

  //differntialDrive for Arcade Drive
  public DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  
  //navX object
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry m_odometry;

  /** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

  /**
  * Creates a new DriveSubsystem.
  //TODO: check if encoder conversions are correct and re charcterized robot
  */ 
  public DriveSubsystem() {
    
    //SETTINGS for TalonFXs

    //setting motors to factory defualt to prevent unexspected behavior
    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    rearLeftMotor.configFactoryDefault();
    rearRightMotor.configFactoryDefault();
    //select falcons integrated encoders
    frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //make rear motor controllers slave to the masters (front controllers)
    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);
    //adjust the direction of the motors
    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(true);
    //have slaves be the same inversion as their corosponding masters
    rearLeftMotor.setInverted(InvertType.FollowMaster);
    rearRightMotor.setInverted(InvertType.FollowMaster);
    //sets motors to neutral
    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    rearLeftMotor.setNeutralMode(NeutralMode.Coast);
    rearRightMotor.setNeutralMode(NeutralMode.Coast);
    //Setting deadband(area required to start moving the motor) to 1%
    frontLeftMotor.configNeutralDeadband(0.01);
    frontRightMotor.configNeutralDeadband(0.01);
    rearLeftMotor.configNeutralDeadband(0.01);
    rearRightMotor.configNeutralDeadband(0.01);
    //Sets voltage compensation to 12, used for percent output
    frontLeftMotor.configVoltageCompSaturation(12);
    frontRightMotor.configVoltageCompSaturation(12);
    rearLeftMotor.configVoltageCompSaturation(12);
    rearRightMotor.configVoltageCompSaturation(12);
    frontLeftMotor.enableVoltageCompensation(true);
    frontRightMotor.enableVoltageCompensation(true);
    rearLeftMotor.enableVoltageCompensation(true);
    rearRightMotor.enableVoltageCompensation(true);
    /** 
    * Setting input side current limit (amps)
    * 45 continious, 80 peak, 30 millieseconds allowed at peak
    * 40 amp breaker can support above 40 amps for a little bit
    * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
    */
    frontLeftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));
    frontRightMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));
    rearLeftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));
    rearRightMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));
    //ramps up motors during Open Loop
    frontLeftMotor.configOpenloopRamp(0.2);
    frontRightMotor.configOpenloopRamp(0.2);
    rearLeftMotor.configOpenloopRamp(0.2);
    rearRightMotor.configOpenloopRamp(0.2);
    //disable ramp up during Closed Loop
    frontLeftMotor.configClosedloopRamp(0);
    frontRightMotor.configClosedloopRamp(0);
    rearLeftMotor.configClosedloopRamp(0);
    rearRightMotor.configClosedloopRamp(0);

    //create odometry object
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    //reset the encoders
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(),
                                                            getRightDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftRate(), getRightRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
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

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  /**
   * 
   * @return the Light encoder in raw units (2048)
   */
  public double rawLeftEncoder(){
    return frontLeftMotor.getSelectedSensorPosition() * (DriveConstants.leftEncoderisReversed ? -1.0 : 1.0);
  }

  /**
   * 
   * @return the right encoder in raw units (2048)
   */
  public double rawRightEncoder(){
    return frontRightMotor.getSelectedSensorPosition() * (DriveConstants.rightEncoderisReversed ? -1.0 : 1.0);
  }

  /**
   * resets the encoders to zero
   */
  public void resetEncoders(){
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }
  
  /**
   * 
   * @return the distance in meters how far the right side has traveled
   */
  public double getRightDistance(){
    return ((rawRightEncoder()/2048.0)/DriveConstants.driveRatio)*(2.0*Math.PI*0.0762);
  }

  /**
   * 
   * @return the distance in meters how far the right side has traveled
   */
  public double getLeftDistance(){
    return ((rawLeftEncoder()/2048.0)/DriveConstants.driveRatio)*(2.0*Math.PI*0.0762); //2piRadius of wheel in meters
  }

  /**
   * @return the rate in meters the right side is going (100ms update?)
   */
  public double getRightRate(){
    return ((frontRightMotor.getSelectedSensorVelocity()/2048.0)/DriveConstants.driveRatio)*(2.0*Math.PI*0.0762);
  }

  /**
   * 
   * @return the rate in meters the left side is going (100ms update?)
   */
  public double getLeftRate(){
    return ((frontLeftMotor.getSelectedSensorVelocity()/2048.0)/DriveConstants.driveRatio)*(2.0*Math.PI*0.0762);
  }

  /**
   * stops all drivetrain motors
   */
  public void stopDriveTrain(){
    frontLeftMotor.stopMotor();
    frontRightMotor.stopMotor();
    rearLeftMotor.stopMotor();
    rearRightMotor.stopMotor();
  }

  public void setSafetyEnabledForAll(){
    frontRightMotor.setSafetyEnabled(true);
    frontLeftMotor.setSafetyEnabled(true);
    rearLeftMotor.setSafetyEnabled(true);
    rearRightMotor.setSafetyEnabled(true);
  }

  public void setSafetyDisabledForAll(){
    frontRightMotor.setSafetyEnabled(false);
    frontLeftMotor.setSafetyEnabled(false);
    rearLeftMotor.setSafetyEnabled(false);
    rearRightMotor.setSafetyEnabled(false);
  }
  

}
