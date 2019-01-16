package org.usfirst.frc.team3928.robot;

import java.lang.reflect.Array;
import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;

public class Odometry {

	public static AHRS navX;	
	public static double positionX;
	public static double positionY;
	//Average of the distance travelled by the two wheels since the last time it was calculated
	static double totalDistance = 0;
	//Distance travelled by the left wheel, as found by the previous calculation
	static double formerLeftDistance = 0;
	//Distance travelled by the right wheel, as found by the previous calculation
	static double formerRightDistance = 0;
	//Distance travelled by the left wheel since the previous calculation
	static double currentLeftDistance = 0;
	//Distance travelled by the left wheel since the previous calculation
	static double currentRightDistance = 0;
	
	public static void calculateDistance(){

		currentRightDistance = Drive.rightEncoder.getDistance()-formerRightDistance;
		currentLeftDistance = Drive.leftEncoder.getDistance()-formerLeftDistance;
		formerLeftDistance = Drive.leftEncoder.getDistance();
		formerRightDistance = Drive.rightEncoder.getDistance();
		totalDistance = (currentLeftDistance + currentRightDistance)/2;
		positionX += totalDistance * Math.sin(navX.getYaw());
		positionY += totalDistance * Math.cos(navX.getYaw());
		System.out.println(positionX + ", " + positionY);

	}
	
}
