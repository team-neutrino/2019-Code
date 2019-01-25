/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

/**
 * Add your docs here.
 * 
 * @author Team Neutrino
 * 
 */
public class NavXTest implements Runnable(){
    private Drive Drive;

    public void run(){
        
        Drive.getNavx.reset();
        System.out.println(Drive.getNavx().getYaw());

    }

}