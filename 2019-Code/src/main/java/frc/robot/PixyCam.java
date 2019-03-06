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
 * This class only tracks one object at a time.
 * 
 * @author JoelNeppel
 *
 */
public class PixyCam implements Runnable
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
	 * The x coordinate of the object's center
	 */
	private int x;
	
	/**
	 * The y coordinate of the object's center
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
		pixyConnection = new SPI(Constants.PixyCam.PIXY_PORT);
		pixyConnection.initAuto(Constants.PixyCam.PIXY_BUFFER_SIZE);
		pixyConnection.setMSBFirst();
		pixyConnection.setClockRate(Constants.PixyCam.PIXY_CLOCKRATE);
		pixyConnection.setClockActiveHigh();
	
		new ValuePrinter(()->
			{
				SmartDashboard.putBoolean("Pixy Tracking: ", isTracking());
				SmartDashboard.putNumber("Pixy X: ", x);
				SmartDashboard.putNumber("Pixy Y :", y);
				SmartDashboard.putNumber("Pixy Width: ", width);
				SmartDashboard.putNumber("Pixy Height: ", height);
			}, 
			ValuePrinter.HIGH_PRIORITY);
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
	 * @return
	 * 	True if the pixy has sent non-zero, accurate data within
	 * 	the last 100 milliseconds, false otherwise
	 */
    public boolean isTracking()
    {
        return checkSum != 0 && checkData() && System.currentTimeMillis() - timeGot < 100;
	}
	
	/**
	 * Converts two bytes into an int.
	 * @param bytes
	 * 	The bytes being converted into an int
	 * @return
	 * 	The int created from the bytes
	 */
	private int bytesToInt(byte[] bytes)
	{
		return (bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF);
	}
	
	@Override
	public void run() 
	{	
		try
		{
			while(!Thread.interrupted())
			{
				byte[] bytes = new byte[2];

				//Look for start bytes
				boolean startFound = false;
				while(!startFound)
				{
					Thread.sleep(1);

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
				checkSum = bytesToInt(bytes);
				
				pixyConnection.read(true, bytes, 2);
				signature = bytesToInt(bytes);
				
				//Switch x with y and width with height since pixy is rotated 90
				pixyConnection.read(true, bytes, 2);
				y = bytesToInt(bytes);
				
				pixyConnection.read(true, bytes, 2);
				x = bytesToInt(bytes);
				
				pixyConnection.read(true, bytes, 2);
				height = bytesToInt(bytes);

				pixyConnection.read(true, bytes, 2);
				width = bytesToInt(bytes);
				
				timeGot = System.currentTimeMillis();

				Thread.sleep(1);
			}
		}
		catch(InterruptedException e)
		{
			//Do nothing - let the thread end
		}			
	}
}