/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private CANSparkMax topShooter = new CANSparkMax(DriveConstants.kTopShooter, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax bottomShooter = new CANSparkMax(DriveConstants.kBottomShooter, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  public CANEncoder topEncoder = new CANEncoder(topShooter);
  public CANEncoder bottomEncoder = new CANEncoder(bottomShooter);

  private CANPIDController mPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


  public ShooterSubsystem() {
    topShooter.setInverted(true);
    bottomShooter.follow(topShooter, true);

    //set up PID
    mPIDController = topShooter.getPIDController();
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    mPIDController.setP(kP);
    mPIDController.setI(kI);
    mPIDController.setD(kD);
    mPIDController.setIZone(kIz);
    mPIDController.setFF(kFF);
    mPIDController.setOutputRange(kMinOutput,kMaxOutput);

  }

  public void startShooter(double joystick) {
    topShooter.set(joystick);
    SmartDashboard.putNumber("top RPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("bottom RPM", bottomEncoder.getVelocity());

  }

  public void velocityShooter(double trigger){
    double setpoint = trigger*maxRPM;
    mPIDController.setReference(setpoint, ControlType.kVelocity);
  }

  /**
   * 
   * @return the RPM of the top shooter motor
   */
  public double getTopEncoderVelocity(){
    return topEncoder.getVelocity();
  }

  /**
   * 
   * @return the RPM of the bottom shooter motor
   */
  public double getBottomEncoderVelocity(){
    return bottomEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
