/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ClimberSubsystem extends SubsystemBase {
  //prototype motor controllers
  private Spark climbMotor = new Spark(DriveConstants.kClimbMotor);
  private Spark elevatormotorPWM = new Spark(DriveConstants.kElevatorPWM);

  //can Talon SRX with encoder motor controller for elavator
  private TalonSRX elevatormotorCAN = new TalonSRX(DriveConstants.kElevatorCAN);

  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    elevatormotorCAN.configFactoryDefault();
    //set brake mode
    elevatormotorCAN.setNeutralMode(NeutralMode.Brake);
    /* Config the sensor used for Primary PID and sensor direction */
    elevatormotorCAN.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    //config peak outputs
    elevatormotorCAN.configPeakOutputForward(1);
    elevatormotorCAN.configPeakOutputReverse(-1);
    //config PID values
    elevatormotorCAN.config_kP(0, 0.001);
    elevatormotorCAN.config_kI(0, 0.0);
    elevatormotorCAN.config_kD(0, 0.0);
    elevatormotorCAN.config_kF(0, 0.0);

    elevatormotorCAN.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", elevatormotorCAN.getSelectedSensorPosition());
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
    elevatormotorPWM.set(Math.abs(0.5)*(DriveConstants.kElevatorIsReversed ? -1.0 : 1.0));
  }
  /**
   * extends the elevator, power based on joystick value
   * @param joystick 0-1 input % for climb motor
   */
  public void extendElevator(double joystick){
    elevatormotorPWM.set(Math.abs(joystick)*(DriveConstants.kElevatorIsReversed ? -1.0 : 1.0));
  }
  /**
   * extends the elevator, power based on joystick value
   * @param joystick 0-1 input % for climb motor
   */
  public void retractElevator(double joystick){
    elevatormotorPWM.set(Math.abs(joystick)*(DriveConstants.kElevatorIsReversed ? 1.0 : -1.0));
  }

  /**
   * extend elevator via PID control on Talon SRX
   */
  public void extendElevatorPID(){
    elevatormotorCAN.set(ControlMode.Position, DriveConstants.kSetPositionClimb);
    SmartDashboard.putNumber("PIDerror", elevatormotorCAN.getClosedLoopError(0));
    SmartDashboard.putNumber("Elevator Position", elevatormotorCAN.getSelectedSensorPosition());
  }

  /**
   * extend elevator via joystick control on Talon SRX
   */
  public void extendElevatorCAN(double joystick){
    elevatormotorCAN.set(ControlMode.PercentOutput, joystick);
    SmartDashboard.putNumber("Elevator Position", elevatormotorCAN.getSelectedSensorPosition());
  }
}
