package frc.robot.utils;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.*;

/**
 * Wrapper Class for Pixy 2 Cam
 * 
 * @author John Trager
 */
public class PixyCam {

    private final Pixy2 pixy;
    private Block mBiggestBlock = null;
    private Block mClosestBlock = null;

    //constants
    private static final double kHorizontalFOV = 60.0;
    private static final double kVerticalFOV = 40.0;
    private static final int kFocalLength = 3;
    private static final double kFrameWidthInPixels = 315.0;
    private static final double kFrameHeightInPixels = 207.0;
    

    public PixyCam(Link link) {
		pixy = Pixy2.createInstance(link);
        pixy.init();
        pixy.setLamp((byte) 1, (byte) 1);
        //skyblue color
        pixy.setLED(135,206,235);
	}

	public PixyCam(Link link, int arg) {
		pixy = Pixy2.createInstance(link);
        pixy.init(arg);
        pixy.setLamp((byte) 1, (byte) 1);
        //skyblue color
        pixy.setLED(135,206,235);
	}

	public Pixy2 getPixy() {
		return pixy;
    }

    public void setLamp(byte x, byte y){
        pixy.setLamp((byte) x, (byte) y);
    }

    public void setLED(int x, int y, int z){
        pixy.setLED(x, y, z);
    }

    /**
     * Gets the biggest block
     * 
     * @return the object that is the widest
     */
    private void getBiggestBlock(){
        // Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
        // does not wait for new data if none is available,
		// and limits the number of returned blocks to 6, for a slight increase in efficiency
        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 6);
        System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
            System.err.println("No block count");
			mBiggestBlock =  null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			    }
            } mBiggestBlock = largestBlock;
    }

    /**
     * Gets the closest block
     * 
     * @return the object that has the highest Y coordinate
     */
    public Block getClosestBlock(){
        // Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
        // does not wait for new data if none is available,
		// and limits the number of returned blocks to 6, for a slight increase in efficiency
        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 6);
        System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
        if (blockCount <= 0) {
            System.err.println("No block count");
            mClosestBlock =  null; // If blocks were not found, stop processing
            return mClosestBlock;
        }
        ArrayList<Block> blocks = pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2
        for (Block block : blocks) {
            if (mClosestBlock==null){
                mClosestBlock = block;
            } else if (block.getY() > mClosestBlock.getY() && block.getAge() > 12){
                mClosestBlock = block;
            }
        }
        
        return mClosestBlock;
    }

    /**
     * 
     * @param block input block
     * @return the angle of the target on X axis (in degrees) from the center of the frame
     */
    public double getPxAngle(Block block){
        return block.getX() * (kHorizontalFOV/kFrameWidthInPixels)-(kHorizontalFOV/2);
    }

     /**
     * 
     * @param block input block
     * @return the angle of the target on Y axis (in degrees) from the center of the frame 
     */
    public double getPyAngle(Block block){
        return (block.getX() * (kVerticalFOV/kFrameHeightInPixels)-(kVerticalFOV/2))*-1;
    }

    public int getNumBlocks(){
        return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 6);
    }

    /**
     * 
     * @return returns if block(s) are in frame and detected
     */
    public boolean isBlock(){
        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 6);
        if (blockCount > 0){
            return true;
        } else {
            return false;
        }
    }

}