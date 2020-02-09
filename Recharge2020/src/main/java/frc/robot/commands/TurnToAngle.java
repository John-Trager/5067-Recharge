/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.toAngleConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /**
   * Turns to desired angle
   *
   * @param m_drive  The drive subsystem
   * @param setAngle Angle the robot is commanded to turn to
   * @param joystick value for driving back/forward
   */
  
  AHRS ahrs;

  public TurnToAngle(DriveSubsystem m_drive, double setAngle, double joystick) {
    super(
        // The controller that the command will use
        new PIDController(toAngleConstants.kP, toAngleConstants.kI, toAngleConstants.kD),
        // This should return the measurement
        () -> m_drive.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> setAngle,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(joystick, output);
        }, m_drive);
    
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(toAngleConstants.kTurnToleranceDeg, toAngleConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
