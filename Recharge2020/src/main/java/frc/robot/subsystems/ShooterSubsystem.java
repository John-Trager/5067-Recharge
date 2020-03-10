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

  private CANEncoder topEncoder = new CANEncoder(topShooter);
  private CANEncoder bottomEncoder = new CANEncoder(bottomShooter);

  private CANPIDController mPIDControllerTop;
  private CANPIDController mPIDControllerBottom;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, bottomSetpointRPM, topSetpointRPM, topError, bottomError;

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

    //set up PID 
    topSetpointRPM = 4000;
    bottomSetpointRPM = 4000;

    mPIDControllerTop = topShooter.getPIDController();
    mPIDControllerBottom = bottomShooter.getPIDController();
    kP = 0.00007; 
    kI = 0.000000265; //to 5600rpm in 1sec; to 4000rpm 3secs -- .0000003 works well for 4000rpm
    kD = 0.000001; 
    kIz = 0; 
    kFF = 0.00008; // ~3000 rpm
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    mPIDControllerTop.setP(kP);
    mPIDControllerTop.setI(kI);
    mPIDControllerTop.setD(kD);
    mPIDControllerTop.setIZone(kIz);
    mPIDControllerTop.setFF(kFF);
    mPIDControllerTop.setOutputRange(kMinOutput,kMaxOutput);
    //bottom pid
    mPIDControllerBottom.setP(kP);
    mPIDControllerBottom.setI(kI);
    mPIDControllerBottom.setD(kD);
    mPIDControllerBottom.setIZone(kIz);
    mPIDControllerBottom.setFF(kFF);
    mPIDControllerBottom.setOutputRange(kMinOutput,kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Top setpoint", topSetpointRPM);
    SmartDashboard.putNumber("Bottom setpoint", bottomSetpointRPM);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("top RPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("bottom RPM", bottomEncoder.getVelocity());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double topSetpointDash = SmartDashboard.getNumber("Top setpoint", 0);
    double bottomSetpointDash = SmartDashboard.getNumber("Bottom setpoint", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    //topPID
    if((p != kP)) { mPIDControllerTop.setP(p); kP = p; }
    if((i != kI)) { mPIDControllerTop.setI(i); kI = i; }
    if((d != kD)) { mPIDControllerTop.setD(d); kD = d; }
    if((iz != kIz)) { mPIDControllerTop.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { mPIDControllerTop.setFF(ff); kFF = ff; }
    if((topSetpointDash != topSetpointRPM)) { topSetpointRPM = topSetpointDash; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      mPIDControllerTop.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;
    }
    //bottom PID
    if((p != kP)) { mPIDControllerBottom.setP(p); kP = p; }
    if((i != kI)) { mPIDControllerBottom.setI(i); kI = i; }
    if((d != kD)) { mPIDControllerBottom.setD(d); kD = d; }
    if((iz != kIz)) { mPIDControllerBottom.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { mPIDControllerBottom.setFF(ff); kFF = ff; }
    if((bottomSetpointDash != bottomSetpointRPM)) { bottomSetpointRPM = bottomSetpointDash; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      mPIDControllerBottom.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;
    }
  }
    

  /**
   * runs the shooter from -1 to 1 (full reverse to full forward)
   * @param joystick input value from -1 to 1
   */
  public void startShooter(double joystick) {
    bottomShooter.set(joystick);
    topShooter.set(joystick);
  }

  public void stopShooterMotors() {
  bottomShooter.stopMotor();
	topShooter.stopMotor();
  
  }

  /**
   * 
   * @param setpoint desired RPM of top motor
   * @param bottomSetpointRPM desired RPM of bottom motor
   */
  public void velocityShooter(double topSetpointRPM, double bottomSetpointRPM){
    mPIDControllerTop.setReference(topSetpointRPM, ControlType.kVelocity);
    mPIDControllerBottom.setReference(bottomSetpointRPM, ControlType.kVelocity);

    topError = topSetpointRPM - topEncoder.getVelocity();
    bottomError = bottomSetpointRPM - bottomEncoder.getVelocity();

  }

  /**
   * FOR TESTING
   * runs both motors at set RPM
   */
  public void velocityShooter(){

    mPIDControllerTop.setReference(topSetpointRPM, ControlType.kVelocity);
    mPIDControllerBottom.setReference(bottomSetpointRPM, ControlType.kVelocity);

    topError = topSetpointRPM - topEncoder.getVelocity();
    bottomError = bottomSetpointRPM - bottomEncoder.getVelocity();
  }

  //TODO: TEST following methods
  
  /**
   * 
   * @return distance to targer in inches
   */
  public double distanceToTarget(){
    //TODO: make equation that bases Ty to distance using stats
    return Limelight.getTy();
  }

  /**
   * 
   * @return rpm for Top motor based on distance in inches
   */
  public static long distanceToTopRPM(double inches){
    //TODO: make equations to calculate rpm based on distance to target in feet
    return Math.round(0.01204921*Math.pow(inches, 2) + -4.5658176*inches + 3133.20148);
  }

  /**
   * 
   * @return rpm for Botttom motor based on distance in inches
   */
  public static long distanceToBottomRPM(double inches){
    //TODO: make equations to calculate rpm based on distance to target in feet
    return Math.round(-2.854E-04*Math.pow(inches,3) + 0.25132007*Math.pow(inches,2) + -69.03675*inches + 8648.40738);
  }

  /**
   * 
   * @param setpoint desired RPM of top motor
   * @param bottomSetpointRPM desired RPM of bottom motor
   */
  public void distanceVelocityShooter(){
    mPIDControllerTop.setReference(distanceToTopRPM(distanceToTarget()), ControlType.kVelocity);
    mPIDControllerBottom.setReference(distanceToBottomRPM(distanceToTarget()), ControlType.kVelocity);

    topError = topSetpointRPM - topEncoder.getVelocity();
    bottomError = bottomSetpointRPM - bottomEncoder.getVelocity();
  }

}
