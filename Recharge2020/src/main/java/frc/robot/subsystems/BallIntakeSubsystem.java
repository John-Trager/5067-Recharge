/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class BallIntakeSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(DriveConstants.kIntakeMotor);
  
  private DoubleSolenoid leftPneumatic = new DoubleSolenoid(0, 1);
  private DoubleSolenoid rightPneumatic = new DoubleSolenoid(2, 3);
  
  //private Compressor compressor = new Compressor(0);
  
  //private double compressorCurrent = compressor.getCompressorCurrent();


  /**
   * Creates a new BallIntakeSubsystem.
   */
  public BallIntakeSubsystem() {

    //setup parameters for intake motor
    intakeMotor.configFactoryDefault();
    intakeMotor.configNeutralDeadband(0.01);
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.configOpenloopRamp(0.5);
    intakeMotor.configVoltageCompSaturation(12);
    intakeMotor.enableVoltageCompensation(true);
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));

    retractIntake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * runs the intake [-1,1]
   */
  public void intakeSetSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

   /**
   * runs intake In based on joystick value
   * @param joystick 0-1 input power % for intake speed
   */
  public void intakeIn(double joystick){
    intakeMotor.set(ControlMode.PercentOutput, Math.abs(joystick)*(DriveConstants.kIntakeIsReversed ? -1.0 : 1.0));
  }

  /**
   * runs intake Out based on joystick value
   * @param joystick 0-1 input power % for intake speed
   */
  public void intakeOut(double joystick){
    intakeMotor.set(ControlMode.PercentOutput, Math.abs(joystick)*(DriveConstants.kIntakeIsReversed ? 1.0 : -1.0));
  }

  public void stopIntakeMotor(){
    intakeMotor.stopMotor();
  }

  /**
   * extends both right and left solenoids
   */
  public void extendIntake(){
    leftPneumatic.set(Value.kForward);
    rightPneumatic.set(Value.kForward);
  }

  /**
   * retracts both right and left solenoids
   */
  public void retractIntake(){
    leftPneumatic.set(Value.kReverse);
    rightPneumatic.set(Value.kReverse);
  }

  /**
   * turns both right and left solenoids off (neither output activated)
   */
  public void turnOffSolenoids(){
    leftPneumatic.set(Value.kOff);
    rightPneumatic.set(Value.kOff);
  }

}
