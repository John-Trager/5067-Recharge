/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class BallIndexSubsystem extends SubsystemBase {
  
  WPI_TalonSRX midIndexMotor = new WPI_TalonSRX(DriveConstants.kMidIndexMotor);
  WPI_TalonSRX backIndexMotor = new WPI_TalonSRX(DriveConstants.kBackIndexMotor);

  // TODO: add ultra sonic sensors

  /**
   * Creates a new BallIndexSubsystem.
   */
  public BallIndexSubsystem() {
   
    //setup parameters for Index motors
    midIndexMotor.configFactoryDefault();
    backIndexMotor.configFactoryDefault();

    midIndexMotor.configNeutralDeadband(0.01);
    backIndexMotor.configNeutralDeadband(0.01);

    midIndexMotor.setNeutralMode(NeutralMode.Brake);
    backIndexMotor.setNeutralMode(NeutralMode.Brake);

    midIndexMotor.configOpenloopRamp(0);
    backIndexMotor.configOpenloopRamp(0);

    midIndexMotor.configVoltageCompSaturation(12);
    backIndexMotor.configVoltageCompSaturation(12);

    midIndexMotor.enableVoltageCompensation(true);
    backIndexMotor.enableVoltageCompensation(true);

    midIndexMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));
    backIndexMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * runs the middle indexer from [1,-1] full forward to full reverse
   */
  public void runMidIndexer(double power){
    midIndexMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * runs the middle indexer from [1,-1] full forward to full reverse
   */
  public void runBackIndexer(double power){
    backIndexMotor.set(ControlMode.PercentOutput, power);
  }
}
