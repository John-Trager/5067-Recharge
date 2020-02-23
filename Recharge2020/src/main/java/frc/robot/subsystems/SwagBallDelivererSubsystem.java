/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwagBallDelivererSubsystem extends SubsystemBase {
  /**
   * Creates a new swagBallDelivererSubsystem.
  */

  Spark swagLift = new Spark(0);
  
  public SwagBallDelivererSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double joystick){
    swagLift.set(joystick*0.5);
  }

  public void runUp(){
    swagLift.set(0.4);
    
  }

  public void runDown(){
    swagLift.set(-0.4);
  }
}
