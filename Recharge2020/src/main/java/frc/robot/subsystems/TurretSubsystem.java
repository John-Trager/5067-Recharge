/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TurretSubsystem extends SubsystemBase {
  WPI_TalonSRX turretMotor = new WPI_TalonSRX(DriveConstants.kTurretMotor);
  /**
   * Creates a new TurretSubsystem.
   * todo:
   * add PID for turrent to turn to position which will be called as an angle from -180-180 ?
   */
  public TurretSubsystem() {
    //config encoder
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
