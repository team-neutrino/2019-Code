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
	/**
	 * The default instance of the NetworkTable
	 */
	private NetworkTableInstance instance = NetworkTableInstance.getDefault();

	/**
	 * A NetworkTable (key is "networkingTest")
	 */
	private NetworkTable table = instance.getTable("networkingTest");
	
	/**
	 * An empty array of bytes
	 */
	private byte[] defaultBytes = new byte[] {};
	
	/**
	 * Sets the value of an entry in the NetworkTable
	 * @param name
	 * The entry to set
	 * @param input
	 * The value to set
	 */
	public void sendBytes(String name, byte[] input) 
	{
		table.getKeys().add(name); 
		
		table.getEntry(name).setRaw(input);
	}
	
	/**
	 * Gets the value of a NetworkTable entry
	 * @param entry
	 * The entry to pull the value from
	 * @return
	 * The value of the specified NetworkTable entry
	 */
	public byte[] getBytes(String entry) 
	{
		System.out.println("Returned " + Arrays.toString(table.getEntry(entry).getRaw(defaultBytes)));
		return table.getEntry(entry).getRaw(defaultBytes);
	}
}