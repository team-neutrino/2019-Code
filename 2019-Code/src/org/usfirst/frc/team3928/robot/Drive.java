package org.usfirst.frc.team3928.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive implements Printer
{
	private Encoder leftEncoder;
	private Encoder rightEncoder;

	public Drive()
	{
		leftEncoder = new Encoder(0, 1);
		rightEncoder = new Encoder(2, 3);

	}
	
	@Override
	public void putValues() 
	{
		SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
	}
}