/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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

    /**
     * Tests the given encoder and reports problems to driver station
     * 
     * Checks rate, needs to be rotating during test.
     * @param e
     *  The encoder to be tested
     * @param name
     *  The name of the encoder
     */
    public static void testEncoder(Encoder e, String name)
    {
        if(e.getRate() == 0)
        {
            DriverStation.reportError("Encoder " + name + " is disconnected.", false);
        }
    }

    /**
     * Tests device for being connected by checking if given result equals zero and reports and error.
     * 
     * Motor needs to be turned on before the test begins.
     * @param m
     *  The TalonSRX to test
     * @param name
     *  The name of the motor
     */
    public static void testTalonSRX(TalonSRX m, String name)
    {
        if(m.getOutputCurrent() == 0)
        {
            DriverStation.reportError("Motor " + name + " with ID " + m.getDeviceID() + " is disconnected.", false);
        }
        Faults f = new Faults();
        m.getFaults(f);
        if(f.UnderVoltage)
        {
            DriverStation.reportError("Motor " + name + " with ID " + m.getDeviceID() + " is under voltage.", false);
        }
        if(f.HardwareFailure)
        {
            DriverStation.reportError("Motor " + name + " with ID " + m.getDeviceID() + " has hardware failure.", false);
        }
    }

    /**
     * Tests solenoid for being blacklisted
     * @param s
     *  The solenoid being tested
     * @param name
     *  The name for the solenoid
     * @param fwdCh
     *  The forward channel
     * @param revCh
     *  The reverse channel
     */
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