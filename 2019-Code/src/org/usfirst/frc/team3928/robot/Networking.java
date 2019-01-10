package org.usfirst.frc.team3928.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.usfirst.frc.team3928.robot.Robot;

public class Networking {	
	
	public static NetworkTableInstance instance = NetworkTableInstance.create();
	public static NetworkTableEntry entry = new NetworkTableEntry(instance, 5800);
	private static byte[] defaultArray = new byte[0];
	
	public static void sendBytes(byte[] bytes) {
		
		entry.setRaw(bytes);
		
	}
	
	public static byte[] getBytes() {
		
		return entry.getRaw(defaultArray);
	
	}

}
