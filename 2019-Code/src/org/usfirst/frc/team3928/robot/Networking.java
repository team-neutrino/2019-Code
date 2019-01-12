package org.usfirst.frc.team3928.robot;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Networking {	
	
	public static NetworkTableInstance instance = NetworkTableInstance.getDefault();
	public static NetworkTable table = instance.getTable("networkingTest");
	
	public static void sendBytes(String name, byte[] input) {
		
		table.getKeys().add("test"); 
		
		table.getEntry("test").setRaw(input);
		
		System.out.println("Set " + Arrays.toString(input));
		
	}
	
	public static byte[] getBytes(String entry) {
		
		System.out.println("Returned " + table);
		return table.getKeys().toString();
		
	}

}
