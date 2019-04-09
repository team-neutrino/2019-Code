/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Controller for the LEDs through the PCM.
 * 
 * Warning: Make sure PCM and LED voltage are the same or add a resistor.
 * 
 * @author JoelNeppel
 * 
 */
public class LEDController
{
    /**
     * Enum for the LED control mode
     */
    public static enum Mode 
    {
        ON, 
        OFF, 
        FLASH,
        MORSE;
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
     * The message flashed when using morse code
     */
    private String message;

    /**
     * The thread for the LED control, only active when in flash or morse mode
     */
    private Thread ledThread;

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
        this.interval = interval;
        this.onTime = onTime;
        this.offTime = offTime;
        this.numPulses = numPulses;
        message = "";
        ledThread = new Thread();

        setMode(mode);
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
        ledThread.interrupt();
        if(mode == Mode.ON)
        {
            ledPort.set(true);
        }
        else if(mode == Mode.OFF)
        {
            ledPort.set(false);
        }
        else if(mode == Mode.FLASH)
        {
            ledThread = new Thread(this::flash);
            ledThread.start(); 
        }
        else if(mode == Mode.MORSE)
        {
            ledThread = new Thread(this::morse);
            ledThread.start(); 
        }
    }

    /**
     * Sets the message to display in morse mode
     * @param morseMessage
     *  The message to set (must contain "-" for long flashes and "." for short 
     *  flashes; other characters will be ignored)
     */
    public void setMessage(String morseMessage)
    {
        message = morseMessage;
    }

    /**
     * Method to set the LEDs to flash.
     */
    private void flash()
    {
        try
        {
            while(!Thread.interrupted())
            {
                for(int i = 0; i < numPulses; i++)
                {
                    ledPort.set(true);
                    Thread.sleep(onTime);
                    ledPort.set(false);
                    Thread.sleep(offTime);
                    Thread.sleep(Math.max(0, interval - offTime));
                }
            }
        }
        catch(InterruptedException e)
        {
            //Do nothing - let the thread end
        }
    }

    /**
     * Method to set the LEDs to morse.
     */
    private void morse()
    {
        try
        {
            while(!Thread.interrupted())
            {
                for(int i = 0; i < message.length(); i++)
                {
                    if(message.charAt(i) == '-')
                    {
                        ledPort.set(true);
                        Thread.sleep(1333);
                        ledPort.set(false);
                        Thread.sleep(1000);
                    }
                    if(message.charAt(i) == '.')
                    {
                        ledPort.set(true);
                        Thread.sleep(667);
                        ledPort.set(false);
                        Thread.sleep(1000);
                    }
                }
            }
        }
        catch(InterruptedException e)
        {
            //Do nothing - let the thread end
        }
    }
}