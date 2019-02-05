/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for the panel transport
 */
public class PanelTransport 
{    
    /**
     * Pushes panels. Name not final.
     */
    private DoubleSolenoid pushers;
   
    /**
     * Holds panels in place. Name not final.
     */
    private DoubleSolenoid holder;

    /**
     * Constructor for panel handler. Port numbers are not final.
     */
    public PanelTransport()
    {
        pushers = new DoubleSolenoid(5, 6);
        holder = new DoubleSolenoid(7, 8);

        new ValuePrinter(()->
        {
            SmartDashboard.putString("Pushers state: ", pushers.get().toString());
        }, 
        ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Turns the pushing solenoid in or out with the given value.
     * @param out
     *  Ture if the cylinder should be out, false if in
     */
    public void setPushersOut(boolean out)
    {
        if(out)
        {
            pushers.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            pushers.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Sets the solenoids to hold the panel.
     * @param hold
     *  True to hold the panel, false to release
     */
    public void setPanelHold(boolean hold)
    {
        if(hold)
        {
            holder.set(DoubleSolenoid.Value.kForward);
        }
        else 
        {
            holder.set(DoubleSolenoid.Value.kReverse);
        }
    }
} 