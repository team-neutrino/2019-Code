/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for the panel transport.
 * 
 * @author Team Neutrino
 * 
 */
public class PanelTransport 
{    
    /**
     * Double solenoid to push the panel out
     */
    private DoubleSolenoid pusher;
   
    /**
     * Double solenoid to hold the panel
     */
    private DoubleSolenoid holder;

    /**
     * Constructor for panel transport.
     */
    public PanelTransport()
    {
        pusher = new DoubleSolenoid(Constants.PanelTransport.PUSHER_CHANNEL_1, Constants.PanelTransport.PUSHER_CHANNEL_2);
        holder = new DoubleSolenoid(Constants.PanelTransport.HOLDER_CHANNEL_1, Constants.PanelTransport.HOLDER_CHANNEL_2);

        // new ValuePrinter(()->
        //     {
        //         SmartDashboard.putString("Pushers state: ", pushers.get().toString());
        //         SmartDashboard.putString("Holder state: ", holder.get().toString());
        //     }, 
        //     ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Turns the pushing solenoid in or out with the given value.
     * @param out
     *  Ture if the cylinders should be out, false if in
     */
    public void setPusherOut(boolean out)
    {
        if(out)
        {
            pusher.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            pusher.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Sets the solenoid to hold or release the panel.
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

    /**
     * Tests systems for panel transport.
     */
    public void systemTest()
    {
        if(pusher.isFwdSolenoidBlackListed())
        {
            DriverStation.reportError("Solenoid " + Constants.PanelTransport.PUSHER_CHANNEL_1 + " is black listed.", false);
        }
    }
} 