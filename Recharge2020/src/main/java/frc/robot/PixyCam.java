package frc.robot;

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
        //// does not wait for new data if none is available,
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

    public double getPx(Block block){
        return block.getX();
    }

    public static double getPy(Block block){
        return block.getY();
    }

    public static double getAge(Block block){
        return block.getAge();
    }

    public static double getWidth(Block block){
        return block.getWidth();
    }

    public boolean isBlock(){
        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 6);
        if (blockCount > 0){
            return true;
        } else {
            return false;
        }
    }

}