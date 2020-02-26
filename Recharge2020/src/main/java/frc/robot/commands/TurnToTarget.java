/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.utils.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends PIDCommand {

  public double lastError;
  /**
   * Turns the robot to vision target
   *
   * @param m_drive  The drive subsystem
   * @param rightJoystick value for driving back/forward
   * 
   */
  public TurnToTarget(DriveSubsystem m_drive, double rightJoystick) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        // This should return the measurement
        () -> Limelight.getTx(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          //m_drive.arcadeDrive(rightJoystick + (Limelight.getTy()*0.025), output+DriveConstants.kF);
          m_drive.arcadeDrive(rightJoystick, output);
        },
        //requires DriveSubsystem
        m_drive);


        
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
