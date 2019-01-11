package org.usfirst.frc.team3928.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Arrays;

import org.usfirst.frc.team3928.robot.Robot;

public class Networking {	
	
	public static NetworkTableInstance instance = NetworkTableInstance.getDefault();
	public static NetworkTableEntry entry = new NetworkTableEntry(instance, 5800);
	public static byte[] defaultArray;
	
	public static void sendBytes(byte[] bytes) {
		
		entry.setRaw(bytes);
		System.out.println("Set " + Arrays.toString(bytes));
		
	}
	
	public static byte[] getBytes() {
		
		System.out.println("Returned " + Arrays.toString(entry.getRaw(defaultArray)));
		
		return entry.getRaw(defaultArray);
		
	}

}
