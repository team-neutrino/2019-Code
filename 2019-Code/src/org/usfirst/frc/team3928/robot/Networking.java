package org.usfirst.frc.team3928.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Arrays;

import org.usfirst.frc.team3928.robot.Robot;

public class Networking {	
	
	public static NetworkTableInstance instance = NetworkTableInstance.getDefault();
	public static NetworkTable table = instance.getTable("networkingTest");
	public static NetworkTableEntry entry = new NetworkTableEntry(instance, 5800);
	public static byte[] defaultArray = new byte[]{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	
	public static void sendBytes(byte[] bytes) {
		
		table.getKeys().add(bytes.toString());
		System.out.println("Set " + bytes.toString());
		
	}
	
	public static byte[] getBytes() {
		
		System.out.println("Returned " + table.getKeys().toString());
		
		return table.getKeys().toString().getBytes();
		
	}

}
