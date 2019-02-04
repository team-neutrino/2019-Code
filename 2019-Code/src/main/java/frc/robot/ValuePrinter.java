/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class used to start a thread that prints values with 
 * a given wait time.
 * 
 * @author NicoleEssner, JoelNeppel
 * 
 */
public class ValuePrinter 
{
    /*
     * The amount to sleep the thread in milliseconds for different priorities.
     */
    public static final int HIGHEST_PRIORITY = 250;
    public static final int HIGH_PRIORITY = 750;
    public static final int NORMAL_PRIORITY = 1000;
    public static final int LOW_PRIORITY = 1250;
    public static final int LOWEST_PRIORITY = 1750;

    /**
     * The time to wait between prints.
     */
    private int waitTime;

    /**
     * Constructor for a value printer that creates a new thread
     * and prints the given values with a wait of the given length
     * between prints.
     * @param printer
     *  The values that will be printed
     * @param timeBetween
     *  The time to wait between prints in milliseconds
     */
    public ValuePrinter(Printer printer, int waitTime)
    {
        this.waitTime = waitTime;

        new Thread(()->
        {
            while(true)
            {
                printer.print();
                Util.threadSleep(this.waitTime);
            }
        }).start();
    }

    /**
     * Changes the time between prints to the given time.
     * @param waitTime
     *  The time to wait between prints in milliseconds
     */
    public void changeWaitTime(int waitTime)
    {
        this.waitTime = waitTime;
    }
}