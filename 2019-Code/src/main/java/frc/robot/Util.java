/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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

    public static void testDisconnect(double result, String name, int deviceID)
    {

    }

    public static void testSolenoid(DoubleSolenoid s, String name, int fwdCh, int revCh)
    {
        s.set(Value.kForward);
        threadSleep(100);
        s.set(Value.kReverse);
        threadSleep(100);
        s.set(Value.kOff);

        if(s.isFwdSolenoidBlackListed())
        {
            DriverStation.reportError("Solenoid " + name + " with channel " + fwdCh + " is blacklisted.", false);
        }
        if(s.isRevSolenoidBlackListed())
        {
            DriverStation.reportError("Solenoid " + name + " with channel " + revCh + " is blacklisted.", false);
        }
    }
}