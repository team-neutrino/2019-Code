/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controller for the LEDs through the PCM.
 * 
 * Warning: Make sure PCM is in 12V mode to prevent damage to LEDs
 * 
 * @author JoelNeppel
 */
public class LEDController implements Runnable, ValuePrinter
{
    /**
     * Enum for the LED control mode
     */
    public static enum Mode 
    {
        ON, 
        OFF, 
        FLASH;
    }

    /**
     * The channel the LEDs are plugged into on the PCM
     */
    private Solenoid ledPort;

    /**
     * The time in milliseconds between flashes
     */
    private int interval;

    /**
     * How long the LEDs should be on during flashes in milliseconds
     */
    private int onTime;

    /**
     * How long the LEDs should be off during flashes in milliseconds
     */
    private int offTime;

    /**
     * The number of pulses each flash cycle
     */
    private int numPulses;

    /**
     * The mode the LEDs are in
     */
    private Mode mode;

    /**
     * Constructor for an LED Controller with the given parameters.
     * @param channel
     *  The PCM channel the LEDs are plugged into
     * @param mode
     *  The initial mode to set the lights to
     * @param interval
     *  The time between the flash cycles in milliseconds
     * @param onTime
     *  The time to keep the LEDs on during flash mode in milliseconds
     * @param offTime
     *  The time to keep the LEDs off between flashes during a cycle in milliseconds
     * @param numPulses
     *  The number of pulses in each flash cycle
     */
    public LEDController(int channel, Mode mode, int interval, int onTime, int offTime, int numPulses)
    {
        ledPort = new Solenoid(channel);
        this.mode = mode;
        this.interval = interval;
        this.onTime = onTime;
        this.offTime = offTime;
        this.numPulses = numPulses;

        new Thread(this).start();
    }

    /**
     * Constructor for an LED Controller with the default flash mode of
     * 1 second on then 1 second off cycle.
     * @param channel
     *  The PCM channel the LEDs are plugged into
     * @param mode
     *  The initial mode to set the lights to
     */
    public LEDController(int channel, Mode controlMode)
    {
        this(channel, controlMode, 1000, 1000, 1000, 1);
    }
    
    /**
     * Constructor for an LED Controller in ON mode with the 
     * default flash mode of 1 second on then 1 second off cycle.
     * @param channel
     *  The PCM channel the LEDs are plugged into
     */
    public LEDController(int channel)
    {
        this(channel, Mode.ON);
    }

    /**
     * Sets the interval between flashes starting with the last pulse 
     * being turned off and the first one be turned on in the next set.
     * @param interval
     *  The interval between flashes in milliseconds
     */
    public void setFlashInterval(int interval)
    {
        this.interval = interval;
    }

    /**
     * Sets the number of flashes in each set.
     * @param pulses
     *  The number of pulses each flash cycle
     */
    public void setFlashPulses(int pulses)
    {
        numPulses = pulses;
    }

    /**
     * Sets the mode of the lights.
     * @param mode
     *  The mode to set the lights to
     */
    public void setMode(Mode mode)
    {
        this.mode = mode;
    }

    @Override
    public void print()
    {
        SmartDashboard.putString("LED State", mode.toString());
    }

    @Override
    public void run()
    {
        while(true)
        {
            if(mode == Mode.FLASH)
            {
                for(int i = 0; i < numPulses; i++)
                {
                    ledPort.set(true);
                    Util.threadSleep(onTime);
                    ledPort.set(false);
                    Util.threadSleep(offTime);
                }
            }
            else if(mode == Mode.ON)
            {
                ledPort.set(true);
            }
            else
            {
                ledPort.set(false);
            }

            Util.threadSleep(Math.max(0, interval - offTime) + 1);
        }
    }
}