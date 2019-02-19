/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class to control pixy in one place with all the useful methods
 * and other needed objects.
 */
public class PixyController 
{
    /**
     * The pixy cam
     */
    private PixyCam pixy;

    /**
     * The white LED controller
     */
    private LEDController whiteLEDs;

    /**
     * Makes pixy cam controller with white LEDs.
     */
    public PixyController()
    {
        pixy = new PixyCam();
        whiteLEDs = new LEDController(0, LEDController.Mode.ON);
    }

    /**
     * Estimates the angle to turn the robot to deliver game pieces
     * using the white lines in front of the bays.
     * @return 
     *  The angle between the robot and bay lines from 0 to 90
     */
    public int estimateAngle()
    {
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

    //TODO track lines
}