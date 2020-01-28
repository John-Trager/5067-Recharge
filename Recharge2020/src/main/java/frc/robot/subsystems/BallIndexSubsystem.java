/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class BallIndexSubsystem extends SubsystemBase {
  Spark frontRollerMotor = new Spark(DriveConstants.kIndexRoller);
  Spark midIndexMotor = new Spark(DriveConstants.kMidIndexMotor);
  Spark backIndexMotor = new Spark(DriveConstants.kBackIndexMotor);

  /**
   * Creates a new BallIndexSubsystem.
   */
  public BallIndexSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
