/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class UltraSonic {

    // A MB1013 distance sensor - http://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf
	// (pins 3, 6 and 7 from sensor to analog input 0)
    private static final AnalogInput mb1013Front = new AnalogInput(0);
    private static final AnalogInput mb1013Rear = new AnalogInput(1);

    // TODO - You will need to determine how to convert voltage to distance
	// (use information from the data sheet, or your own measurements)
    private static final double VOLTS_TO_DIST = 1.0;
    
    public static double getFrontVoltage() {
	    return mb1013Front.getVoltage();
	  }
	  
	public static double getFrontDistance() {
	    return getFrontVoltage() * VOLTS_TO_DIST;
    }
      
    public static double getRearVoltage() {
	    return mb1013Rear.getVoltage();
	}
	  
	public static double getRearDistance() {
	    return getRearVoltage() * VOLTS_TO_DIST;
	}

}
