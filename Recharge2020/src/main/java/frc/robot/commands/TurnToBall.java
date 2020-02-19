/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.PIDtoBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.PixyCam;
import io.github.pseudoresonance.pixy2api.links.SPILink;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToBall extends PIDCommand {
  private static PixyCam pixy = new PixyCam(new SPILink());
  /**
   * Creates a new TurnToBall.
   * @param m_drive  The drive subsystem
   * @param rightJoystick value for driving back/forward
   */
  public TurnToBall(DriveSubsystem m_drive, double rightJoystick) {
    super(
        // The controller that the command will use
        new PIDController(PIDtoBall.kP, PIDtoBall.kI, PIDtoBall.kD),
        // This should return the measurement
        () -> pixy.getPxAngle(pixy.getClosestBlock()),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(rightJoystick, output);
        },
        m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
