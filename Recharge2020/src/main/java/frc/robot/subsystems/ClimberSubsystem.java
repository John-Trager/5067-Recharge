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

public class ClimberSubsystem extends SubsystemBase {
  private Spark climbMotor = new Spark(DriveConstants.kClimbMotor);
  private Spark elevatorMotor = new Spark(DriveConstants.kElevator);
  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * runs climber at 50%
   */
  public void startClimb(){
    climbMotor.set(0.5);
  }
  /**
   * runs climber at based on joystick value
   * @param joystick 0-1 input % for climb motor
   */
  public void startClimb(double joystick){
    climbMotor.set(Math.abs(joystick)*(DriveConstants.kClimberIsReversed ? -1.0 : 1.0));
  }
  /**
   * extends the elevator at 50% power
   */
  public void extendElevator(){
    elevatorMotor.set(Math.abs(0.5)*(DriveConstants.kElevatorIsReversed ? -1.0 : 1.0));
  }
  /**
   * extends the elevator, power based on joystick value
   * @param joystick 0-1 input % for climb motor
   */
  public void extendElevator(double joystick){
    elevatorMotor.set(Math.abs(joystick)*(DriveConstants.kElevatorIsReversed ? -1.0 : 1.0));
  }
}
