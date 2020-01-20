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

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  Spark topShooter = new Spark(DriveConstants.kTopShooter);
  Spark bottomShooter = new Spark(DriveConstants.kBottomShooter);


  public ShooterSubsystem() {
    topShooter.setInverted(false);
    bottomShooter.setInverted(true);
  }

  public void startShooter(double joystick) {
    topShooter.set(joystick);
    bottomShooter.set(joystick);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
