/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * Class for the PixyCam that reads data form the serial port and parses data of the 
 * object being tracked.
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
    
    private long timeGot;
		
	/**
	 * Constructs a pixyCam connected to the serial port with a buffer size of 100.
	 */
	public PixyCam()
	{
		pixyConnection = new SPI(SPI.Port.kOnboardCS0);
		pixyConnection.initAuto(100);
		pixyConnection.setMSBFirst();
		pixyConnection.setClockRate(1000000);
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
	 * @param values
	 * 	The values that are being checked wit the checksum in index 0
	 * @return
	 * 	True if the data is correct, false of the data contains an error
	 */
	public boolean checkData()
	{
		int sum = signature + x + y + width + height;
			
		return sum == checkSum;
    }
    
    public boolean isTracking()
    {
        return checkSum != 0 && System.currentTimeMillis() - timeGot < 100;
    }
	
	@Override
	public void run() 
	{		
		try 
		{
			Thread.sleep(1000);
		} 
		catch (InterruptedException e1) 
		{
			e1.printStackTrace();
		}
		
		while(true)
		{
			byte[] bytes = new byte[2];

			//Look for start bytes
			boolean startFound = false;
			while(!startFound)
			{
				try 
				{
					Thread.sleep(10);
				} 
				catch (InterruptedException e) 
				{
					e.printStackTrace();
				}
				pixyConnection.read(true, bytes, 2);
				
				if(((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF)) == 0xaa55)
				{
					startFound = true;
				}
			// 	System.out.println("looking");
			// 	System.out.printf("0x%02X, ", bytes[0]);
			// 	System.out.printf("0x%02X, ", bytes[1]);
			// 	System.out.printf("0x%02X, ", ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF)));
			 }

			pixyConnection.read(true, bytes, 2);
			//Check if next bytes are start bytes meaning a new frame
			if(((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF)) == 0xaa55)
			{
				pixyConnection.read(true, bytes, 2);

			}
			
			//Set values from bytes
			checkSum = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			//System.out.println(checkSum);
			
			pixyConnection.read(true, bytes, 2);
			signature = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
//			System.out.println("signature: ");
//			System.out.printf("0x%02X, ", bytes[0]);
//			System.out.printf("0x%02X, ", bytes[1]);
//
//			System.out.println(signature);
			
			pixyConnection.read(true, bytes, 2);
			x = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
//			System.out.println("x: ");
//			System.out.printf("0x%02X, ", bytes[0]);
//			System.out.printf("0x%02X, ", bytes[1]);
//
//			System.out.println(x);
			
			pixyConnection.read(true, bytes, 2);
			y = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			//System.out.println(y);
			
			pixyConnection.read(true, bytes, 2);
			width = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			//System.out.println(width);

			pixyConnection.read(true, bytes, 2);
			height = ((bytes[0] & 0xFF) << 8 | (bytes[1] & 0xFF));
			//System.out.println(height);

			//System.out.println(checkData());
            //System.out.println(signature + ", " + x + ", " + y + ", " + width + ", " + height + ", ");
            
            timeGot = System.currentTimeMillis();

			try 
			{
				Thread.sleep(1);
			}
			catch (InterruptedException e) 
			{
				e.printStackTrace();
			}
		}
	}
}