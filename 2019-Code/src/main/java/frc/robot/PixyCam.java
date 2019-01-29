/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for the PixyCam that reads data form the SPI and parses data of the 
 * object being tracked.
 * 
 * @author JoelNeppel
 *
 */
public class PixyCam implements Runnable, ValuePrinter
{
	/**
	 * The pixyCam connection
	 */
	private SPI pixyConnection;
		
	/**
	 * The sum of the signature, x, y, width, and height sent by Pixy
	 */
	private int checkSum;
	
	/**
	 * The object signature
	 */
	private int signature;
	
	/**
	 * The x coordinate of the object
	 */
	private int x;
	
	/**
	 * The y coordinate of the object
	 */
	private int y;
	
	/**
	 * The width of the object
	 */
	private int width;
	
	/**
	 * The height of the object
	 */
  	private int height;
	
	/**
	 * The system time when new data was last processed  
	 */
    private long timeGot;
		
	/**
	 * Constructs a pixyCam connected to the SPI with a buffer size of 100.
	 */
	public PixyCam()
	{
		pixyConnection = new SPI(Constants.PIXY_PORT);
		pixyConnection.initAuto(Constants.PIXY_BUFFER_SIZE);
		pixyConnection.setMSBFirst();
		pixyConnection.setClockRate(Constants.PIXY_CLOCKRATE);
		pixyConnection.setClockActiveHigh();
		
		new Thread(this).start();
	}
	
	/**
	 * Returns the center X coordinate of the object being tracked.
	 * @return
	 * 	The center X value
	 */
	public int getX()
	{
		return x;
	}
	
	/**
	 * Returns the center Y coordinate of the object being tracked.
	 * @return
	 * 	The center Y value
	 */
	public int getY()
	{
		return y;
	}
	
	/**
	 * Returns the width of the object in pixels.
	 * @return
	 * 	The width of the object
	 */
	public int getWidth()
	{
		return width;
	}
	
	/**
	 * Returns the height of the object in pixels.
	 * @return
	 * 	The height of the object
	 */
	public int getHeight()
	{
		return height;
	}

	/**
	 * Checks the values using the checksum in index 0 to verify the data is correct.
	 * @return
	 * 	True if the data is correct, false of the data contains an error
	 */
	public boolean checkData()
	{
		int sum = signature + x + y + width + height;
			
		return sum == checkSum;
    }
	
	/**
	 * Returns whether the pixy is currently tracking an object based
	 * on if the checkSum is 0 and the last time data was processed.
	 * 
	 * @return
	 * 	True if the pixy has sent non-zero data within
	 * 	the last 100 milliseconds, false otherwise
	 */
    public boolean isTracking()
    {
        return checkSum != 0 && System.currentTimeMillis() - timeGot < 100;
    }
	
	@Override
	public void run() 
	{				
		while(true)
		{
			byte[] bytes = new byte[2];

			//Look for start bytes
			boolean startFound = false;
			while(!startFound)
			{
				Util.threadSleep(1);

				pixyConnection.read(true, bytes, 2);
				
				if(((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF)) == 0xaa55)
				{
					startFound = true;
				}
			 }

			pixyConnection.read(true, bytes, 2);
			//Check if next bytes are start bytes meaning a new frame
			if(((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF)) == 0xaa55)
			{
				pixyConnection.read(true, bytes, 2);

			}
			
			//Set values from bytes received
			checkSum = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			
			pixyConnection.read(true, bytes, 2);
			signature = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			
			pixyConnection.read(true, bytes, 2);
			x = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			
			pixyConnection.read(true, bytes, 2);
			y = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			
			pixyConnection.read(true, bytes, 2);
			width = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));

			pixyConnection.read(true, bytes, 2);
			height = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
            
            timeGot = System.currentTimeMillis();

			Util.threadSleep(1);
		}
	}

	@Override
	public void print()
	{
		SmartDashboard.putBoolean("Pixy Tracking", isTracking());
		SmartDashboard.putNumber("X", x);
		SmartDashboard.putNumber("Y", y);
		SmartDashboard.putNumber("Width", width);
        SmartDashboard.putNumber("Height", height);
    }
}