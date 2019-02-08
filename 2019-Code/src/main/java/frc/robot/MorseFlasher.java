/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.LEDController.Mode;

/**
 * Add your docs here.
 */
public class MorseFlasher {

    private LEDController controller;

    /**
     * Flashes a morse code message (- for long, . for short)
     * @param message The message to flash
     * @param repetitions The number of times to flash the message (0 for infinity)
     */
    public void flashMessage(String message, int repetitions)
    {
        controller = new LEDController(Constants.WHITE_LED_PORT, Mode.OFF);
        for(int j = 0; j < repetitions; j++)
        {
            for(int i = 0; i < message.length(); i++)
            {
                if(message.charAt(i) == "-".charAt(0))
                {
                    controller.setMode(Mode.ON);
                    Util.threadSleep(1333);
                    controller.setMode(Mode.OFF);
                    Util.threadSleep(1000);
                }

                if(message.charAt(i) == ".".charAt(0))
                {
                    controller.setMode(Mode.ON);
                    Util.threadSleep(667);
                    controller.setMode(Mode.OFF);
                    Util.threadSleep(1000);
                }
            }
        }
    }
}
