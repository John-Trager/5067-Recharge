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

public class BallIntakeSubsystem extends SubsystemBase {
  Spark intakeMotor = new Spark(DriveConstants.kIntakeMotor);
  /**
   * Creates a new BallIntakeSubsystem.
   */
  public BallIntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * runs intake IN at 50%
   */
  public void intakeIn(){
    intakeMotor.set(0.5);
  }

   /**
   * runs intake In based on joystick value
   * @param joystick 0-1 input power % for intake speed
   */
  public void intakeIn(double joystick){
    intakeMotor.set(Math.abs(joystick)*(DriveConstants.kIntakeIsReversed ? -1.0 : 1.0));
  }

  /**
   * runs intake out at 50%
   */
  public void intakeOut(){
    intakeMotor.set(-0.5);
  }

  /**
   * runs intake Out based on joystick value
   * @param joystick 0-1 input power % for intake speed
   */
  public void intakeOut(double joystick){
    intakeMotor.set(Math.abs(joystick)*(DriveConstants.kIntakeIsReversed ? 1.0 : -1.0));
  }
}
