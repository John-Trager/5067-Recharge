/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Limelight;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax topShooter = new CANSparkMax(DriveConstants.kTopShooter, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax bottomShooter = new CANSparkMax(DriveConstants.kBottomShooter, CANSparkMaxLowLevel.MotorType.kBrushless);

  double kPower;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    topShooter.restoreFactoryDefaults();
    bottomShooter.restoreFactoryDefaults();

    topShooter.enableVoltageCompensation(12);
    bottomShooter.enableVoltageCompensation(12);

    topShooter.setInverted(false);
    bottomShooter.setInverted(true);

    //topShooter.setIdleMode(IdleMode.kBrake);
    kPower = 0.6;
    //set up PID 
    SmartDashboard.putNumber("Shooter Power", kPower);

  }

  @Override
  public void periodic() {
    double dashPower = SmartDashboard.getNumber("Shooter Power", 0);
    if(( dashPower != kPower)) {kPower = dashPower; }
    
  }

  /**
   * runs the shooter from -1 to 1 (full reverse to full forward)
   * @param joystick input value from -1 to 1
   */
  public void startShooter(double joystick) {
    bottomShooter.set(joystick);
    topShooter.set(joystick);
  }

  public void startShooter() {
    bottomShooter.set(kPower);
    topShooter.set(kPower);
  }

public void stopShooterMotors() {
  bottomShooter.stopMotor();
	topShooter.stopMotor();
  
}

}
