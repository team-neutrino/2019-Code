/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class to control pixy in one place with all the useful methods
 * and other needed objects including the LEDs and thread.
 * 
 * @author JoelNeppel
 * 
 */
public class PixyController 
{
    /**
     * The pixy cam
     */
    private PixyCam pixy;

    /**
     * The controller for the white LEDs
     */
    private LEDController leds;

    /**
     * The thread for the pixy cam
     */
    private Thread pixyThread;

    /**
     * Makes pixy cam controller with white LEDs.
     */
    public PixyController()
    {
        pixy = new PixyCam();
        leds = new LEDController(Constants.PixyController.LED_PORT, LEDController.Mode.OFF);
        pixyThread = new Thread(pixy);
    }

    /**
     * Estimates the angle to turn the robot to deliver game pieces
     * using the white lines in front of the bays.
     * @return 
     *  The angle between the robot and bay lines from 0 to 90
     */
    public int estimateAngle()
    {
        //TODO test this
        if(pixy.isTracking())
        {
            double angle = Math.tanh((double) (pixy.getHeight()) / pixy.getWidth());

            return (int) Math.toDegrees(angle);
        }

        return 0;
    }

    /**
     * Returns whether the pixycam is tracking an object or not.
     * @return
     *  True if pixy is tracking an object, false otherwise
     */
    public boolean isTracking()
    {
        return pixy.isTracking();
    }

    /**
     * Starts tracking objects using the pixy cam by starting
     * the pixy thread and turning on the LEDs.
     */
    public void startTracking()
    {
        if(!pixyThread.isAlive())
        {
            pixyThread = new Thread(pixy);
            pixyThread.start();
            leds.setMode(LEDController.Mode.ON);
        }
    }

    /**
     * Stops tracking objects to save system resources and energy
     * by stopping the pixy thread and turning off the LEDs.
     */
    public void stopTracking()
    {
        pixyThread.interrupt();
        leds.setMode(LEDController.Mode.OFF);
    }

    //TODO track lines
}