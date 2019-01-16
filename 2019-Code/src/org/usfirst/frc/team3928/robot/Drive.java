package org.usfirst.frc.team3928.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive implements Printer
{
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	public static TalonSRX talonRight1;
	public static TalonSRX talonRight2;
	public static TalonSRX talonLeft1;
	public static TalonSRX talonLeft2;

	public Drive()
	{
		leftEncoder = new Encoder(2, 3);
		rightEncoder = new Encoder(4, 5);
		talonRight1 = new TalonSRX(0);
		talonRight2 = new TalonSRX(1);
		talonLeft1 = new TalonSRX(2);
		talonLeft2 = new TalonSRX(3);

	}
	
	@Override
	public void putValues() 
	{
		SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
	}
}
