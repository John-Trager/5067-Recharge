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
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ClimberSubsystem extends SubsystemBase {
  //prototype motor controllers
  private WPI_TalonSRX climbMotor = new WPI_TalonSRX(DriveConstants.kClimbMotor);
  //can Talon SRX with encoder motor controller for elavator
  private WPI_TalonSRX elevatormotorCAN = new WPI_TalonSRX(DriveConstants.kElevatorCAN);

  private double kP, kI, kD, kF, setPoint;

  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    climbMotor.configFactoryDefault();
    elevatormotorCAN.configFactoryDefault();
    //set brake mode
    elevatormotorCAN.setNeutralMode(NeutralMode.Brake);
    climbMotor.setNeutralMode(NeutralMode.Brake);
    //set and enable voltage compensation
    elevatormotorCAN.configVoltageCompSaturation(12);
    climbMotor.configVoltageCompSaturation(12);
    elevatormotorCAN.enableVoltageCompensation(true);
    climbMotor.enableVoltageCompensation(true);
    //limitcurrent to motors
    elevatormotorCAN.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));
    climbMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));

    // sets soft limit so lift won't over extend (like a rev limiter lol) and enables it
    //TODO: for comp/final version tweak values including the setpoint
    elevatormotorCAN.configForwardSoftLimitThreshold(DriveConstants.kForwardSoftLimit);
    elevatormotorCAN.configReverseSoftLimitThreshold(DriveConstants.kReverseSoftLimit);
    elevatormotorCAN.configForwardSoftLimitEnable(true);
    elevatormotorCAN.configReverseSoftLimitEnable(true);
    //inverses encoder relative to motor direction
    elevatormotorCAN.setSensorPhase(true);
    /* Config the sensor used for Primary PID and sensor direction */
    //Encoder is 1024 count per revolution
    elevatormotorCAN.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    //reset encoder position
    elevatormotorCAN.setSelectedSensorPosition(0);
    //config peak outputs
    elevatormotorCAN.configNominalOutputForward(0);
    elevatormotorCAN.configNominalOutputReverse(0);
    elevatormotorCAN.configPeakOutputForward(1);
    elevatormotorCAN.configPeakOutputReverse(-1);
    //sets motor ramping
    elevatormotorCAN.configClosedloopRamp(0.5);
    //config PID values
    //TODO: tune via smartDash
    setPoint = DriveConstants.kSetPositionClimb;
    kP = 0.07;
    kI = 0.00001;
    kD = 0.008;
    kF = 0.0;
    elevatormotorCAN.config_kP(0, kP);
    elevatormotorCAN.config_kI(0, kI);
    elevatormotorCAN.config_kD(0, kD);
    elevatormotorCAN.config_kF(0, kF);

    //display values for tweaking
    SmartDashboard.putNumber("P Gain ELE", kP);
    SmartDashboard.putNumber("I Gain ELE", kI);
    SmartDashboard.putNumber("D Gain ELE", kD);
    SmartDashboard.putNumber("F Gain ELE", kF);
    SmartDashboard.putNumber("ELE setPoint", setPoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", elevatormotorCAN.getSelectedSensorPosition());

    double p = SmartDashboard.getNumber("P Gain ELE", 0);
    double i = SmartDashboard.getNumber("I Gain ELE", 0);
    double d = SmartDashboard.getNumber("D Gain ELE", 0);
    double f = SmartDashboard.getNumber("F Zone ELE", 0);
    double setPointDash = SmartDashboard.getNumber("ELE setPoint", setPoint);

    //update PIDF, set point if changed on Dashboard
    if((p != kP)) { elevatormotorCAN.config_kP(0, p); kP = p; }
    if((i != kI)) { elevatormotorCAN.config_kI(0, i); kI = i; }
    if((d != kD)) { elevatormotorCAN.config_kI(0, d); kD = d; }
    if((f != kF)) { elevatormotorCAN.config_kI(0, f); kF = f; }
    if((setPointDash != setPoint)) { elevatormotorCAN.config_kI(0, setPointDash); setPoint = setPointDash; }
  
  }

  /**
   * runs climber winch at 50%
   */
  public void startClimb(){
    climbMotor.set(ControlMode.PercentOutput, 0.5);
  }
  /**
   * runs climber winch based on joystick value
   * @param joystick 0-1 input % for climb motor
   */
  public void startClimb(double joystick){
    climbMotor.set(ControlMode.PercentOutput, Math.abs(joystick)*(DriveConstants.kClimberIsReversed ? 1.0 : -1.0));
  }

  /**
   *  extend elevator via PID to the constant setpoint
   */
  public void extendElevatorPID(){
    elevatormotorCAN.set(ControlMode.Position, setPoint);
    SmartDashboard.putNumber("PIDerror", elevatormotorCAN.getClosedLoopError(0));
  }

  /**
   *  extend elevator via PID control on Talon SRX
   * alternate -> for setting setPoint
   */
  public void extendElevatorPID(double setpoint){
    /*
    elevatormotorCAN.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);
    */
    elevatormotorCAN.set(ControlMode.Position, setpoint);
    SmartDashboard.putNumber("PIDerror", elevatormotorCAN.getClosedLoopError(0));
  }

  /**
   * extend elevator via joystick control on Talon SRX
   */
  public void extendElevatorCAN(double joystick){
    elevatormotorCAN.set(ControlMode.PercentOutput, joystick);
  }

  /**
   * retract elevator via joystick control on Talon SRX
   */
  public void retractElevatorCAN(double joystick){
    //TODO: create PID for lowering the elevator
    if (elevatormotorCAN.getSelectedSensorPosition() > 4000){
      elevatormotorCAN.set(ControlMode.PercentOutput, -0.5*joystick);
    } else {
      elevatormotorCAN.set(ControlMode.PercentOutput, -0.1*joystick);
    }

  }

  /**
   * Retracts the elevator in a semi-controlled manor
   * 
   *  - issa bang-bang loop
   *  - contues retracting until it hits soft limit
   */
  public void retractElevatorCAN(){
    //TODO: tune bang-bang loop values fo retraction

    if (elevatormotorCAN.getSelectedSensorPosition() > 4000){
       elevatormotorCAN.set(ControlMode.PercentOutput, -0.4);
    } else if (elevatormotorCAN.getSelectedSensorPosition() > 0){
      elevatormotorCAN.set(ControlMode.PercentOutput, -0.1);
    } else {
      elevatormotorCAN.stopMotor();
     }
  }

  /**
   * stops elevator motor
   */
  public void stopElevator(){
    elevatormotorCAN.stopMotor();
  }

  /**
   * stops winch motor
   */
  public void stopClimber(){
    climbMotor.stopMotor();
  }
    
}

