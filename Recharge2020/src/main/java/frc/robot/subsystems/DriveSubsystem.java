/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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

  //differntialDrive for Arcade Drive
  //DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

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
    //create odometry object
  
  }

  /*
  void LoadMusicSelection(int offset){
        // increment song selection 
        _songSelection += offset;
        /* wrap song index in case it exceeds boundary 
        if (_songSelection >= _songs.length) {
            _songSelection = 0;
        }
        if (_songSelection < 0) {
            _songSelection = _songs.length - 1;
        }
        /* load the chirp file 
        _orchestra.loadMusic(_songs[_songSelection]); 

        /* print to console 
        System.out.println("Song selected is: " + _songs[_songSelection] + ".  Press left/right on d-pad to change.");
        
        /* schedule a play request, after a delay.  
            This gives the Orchestra service time to parse chirp file.
            If play() is called immedietely after, you may get an invalid action error code. 
        _timeToPlayLoops = 10;
  } */


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

}
