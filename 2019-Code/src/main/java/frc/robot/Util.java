/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Utility class for Thread.sleep() without having a try catch block.
 * 
 * @author NicoleEssner
 * 
 */
public class Util 
{
    /**
     * Sleeps the thread for the given amount of time.
     * @param millis
     *  The number of milliseconds to sleep the thread for
     */
    public static void threadSleep(int millis)
    {
        try 
		{
			Thread.sleep(millis);
		} 
		catch (InterruptedException e) 
		{
			e.printStackTrace();
		}
    }
}