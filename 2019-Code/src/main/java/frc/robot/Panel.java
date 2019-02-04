/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Panel 
{    
    //TODO look into DoubleSolenoid
    
    /**
     * Pushes panels. Name not final.
     */
    private Solenoid push;
   
    /**
     * Holds panels in place. Name not final.
     */
    private Solenoid hold;

    /**
     * Constructor for panel handler. Port numbers are not final.
     */
    public Panel()
    {
        push = new Solenoid(5);
        hold = new Solenoid(7);

        new ValuePrinter(()->
        {
            //TODO print values to SmartDashboard
        }, 
        ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Turns the "push" Solenoid on or off
     */
    public void togglePushSolenoid()
    {
        push.set(!push.get());
    }

    /**
     * Turns the "push" Solenoid on or off
     */
    public void toggleHoldSolenoid()
    {
        hold.set(!hold.get());
    }
}