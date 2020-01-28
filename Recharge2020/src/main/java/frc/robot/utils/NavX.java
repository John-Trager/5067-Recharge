package frc.robot.utils;

//https://www.kauailabs.com/dist/frc/2020/navx_frc.json
//resolve offline library donwload
//currently must be using internet and above url to load library
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
/**
 * Wrapper class for getting navX values
 * 
 * @author John Trager
 */
public class NavX {
    AHRS ahrs;

    public void startNavX() {
        try {
            /***********************************************************************
             * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
             * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
             * 
             * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
             * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
             * 
             * Multiple navX-model devices on a single robot are supported.
             ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex ) {
            //DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
            System.out.println("Error instantiating navX MXP:  " + ex.getMessage());
        }
    }

    /**
    * Gets Angle of Gyro which can be used with WPIlin Gyro class
    * 
    * @return The current total accumulated yaw angle (Z axis) of the robot in degrees. 
    * 
    * This heading is based on integration of the returned rate from the Z-axis (yaw) gyro.
    */
    public double getAngle(){
        return ahrs.getAngle();
    }

    /**
    * Gets Angle rate of Gyro which can be used with WPIlin Gyro class
    * 
    * @return current rate of change in yaw angle (in degrees per second) 
    * 
    * The rate is based on the most recent reading of the yaw gyro angle
    */
    public double getRate(){
        return ahrs.getRate();
    }

    /**
    * Resets Yaw Heading
    * 
    * @author John Trager
    */
    public void zeroYaw(){
        ahrs.zeroYaw();
    }

    /**
    * checks if the IMU is Calibrating
    * 
    *
    * @return Returns true if the sensor is currently automatically calibrating the gyro and accelerometer sensors.
    */
    public boolean isCalibrating(){
        return ahrs.isCalibrating();
    } 
   
    /**
    * checks if the IMU is Connected
    *
    *
    * @return Returns true if a valid update has been recently received from the sensor.
    */
    public boolean isConnected(){
        return ahrs.isConnected();
    }

    /**
    * gets the yaw axis 
    * 
    *
    * @return The current yaw value in degrees (-180 to 180).
    */
    public float getYaw(){
        return ahrs.getYaw();
    }
    
    /**
    * gets the yaw pitch 
    * 
    *
    * @return The current pitch value in degrees (-180 to 180).
    */
    public float getPitch(){
        return ahrs.getPitch();
    } 

    /**
    * gets the roll
    * 
    *
    * @return The current roll value in degrees (-180 to 180).
    */
    public float getRoll(){
        return ahrs.getRoll();
    }
    
    /**
    * gets the compass heading 
    *
    * Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)
    * 
    *
    * @return The current tilt-compensated compass heading, in degrees (0-360).
    */
    public float getCompassHeading(){
        return ahrs.getCompassHeading();
    } 

    /** 
    *  Display 9-axis Heading (requires magnetometer calibration to be useful) 
    *
    *
    * @return Fused Heading in Degrees (range 0-360)
    */
    public float getFusedHeading(){
        return ahrs.getFusedHeading();
    } 


    
}