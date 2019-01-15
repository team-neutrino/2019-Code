package org.usfirst.frc.team3928.robot;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.usfirst.frc.team3928.robot.Networking;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Fak3Drive implements SendableBuilder{
		
	@Override
	public void setSmartDashboardType(String type) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setSafeState(Runnable func) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setUpdateTable(Runnable func) {
		// TODO Auto-generated method stub
		
	}

	public static byte[] getRaw(String key) {
		
		System.out.println("Driver Station sees: " + Arrays.toString(Networking.table.getEntry(key).getRaw(Networking.defaultBytes)));
		return Networking.table.getEntry(key).getRaw(Networking.defaultBytes);
	}
	
	@Override
	public NetworkTableEntry getEntry(String key) {
		System.out.println("Driver Station sees: " + Arrays.toString(Networking.table.getEntry(key).getRaw(Networking.defaultBytes)));
		return Networking.table.getEntry(key);
	}

	@Override
	public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter, Consumer<boolean[]> setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addDoubleArrayProperty(String key, Supplier<double[]> getter, Consumer<double[]> setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addStringArrayProperty(String key, Supplier<String[]> getter, Consumer<String[]> setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addRawProperty(String key, Supplier<byte[]> getter, Consumer<byte[]> setter) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addValueProperty(String key, Supplier<NetworkTableValue> getter, Consumer<NetworkTableValue> setter) {
		// TODO Auto-generated method stub
		
	}

	
	
}
