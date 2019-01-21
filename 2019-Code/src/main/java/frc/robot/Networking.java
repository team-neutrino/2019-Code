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
public class Networking 
{
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private NetworkTable table = instance.getTable("networkingTest");
	private byte[] defaultBytes = new byte[] {};
	
	public void sendBytes(String name, byte[] input) {
		
		table.getKeys().add(name); 
		
		table.getEntry(name).setRaw(input);
		
		System.out.println("Set " + Arrays.toString(input));
		
	}
	
	public byte[] getBytes(String entry) {
		
		System.out.println("Returned " + Arrays.toString(table.getEntry(entry).getRaw(defaultBytes)));
		return table.getEntry(entry).getRaw(defaultBytes);
		
	}
}