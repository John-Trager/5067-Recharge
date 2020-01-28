package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * Wrapper Class for PDP
 * 
 * @author John Trager
 */
public class PDP {
    //delcare PDP
    private static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    /**
     * 
     * @return the robots voltage
     */
    public double getVoltage(){
        return pdp.getVoltage();
    }
    /**
     * 
     * @return the robots PDP temp
     */
    public double getTemp(){
        return pdp.getTemperature();
    }
    /**
     * 
     * @return the robots total current draw
     */
    public double getTotalCurrent(){
        return pdp.getTotalCurrent();
    }
     /**
     * @param chanel of device
     * @return the robots current draw for chanel
     */
    public static double getCurrent(int chanel){
        return pdp.getCurrent(chanel);
    }



    
    





}