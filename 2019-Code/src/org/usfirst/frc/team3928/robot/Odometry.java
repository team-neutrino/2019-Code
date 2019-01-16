package org.usfirst.frc.team3928.robot;

import java.lang.reflect.Array;
import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;

public class Odometry {

	public AHRS navx;	
	public static double positionX;
	public static double positionY;
	//Average of the distance travelled by the two wheels since the last time it was calculated
	double totalDistance = 0;
	//Distance travelled by the left wheel, as found by the previous calculation
	double formerLeftDistance = 0;
	//Distance travelled by the right wheel, as found by the previous calculation
	double formerRightDistance = 0;
	//Distance travelled by the left wheel since the previous calculation
	double currentLeftDistance = 0;
	//Distance travelled by the left wheel since the previous calculation
	double currentRightDistance = 0;
	
	public void calculateDistance(){

		currentRightDistance = Drive.rightEncoder.distance-formerRightDistance;
		currentLeftDistance = Drive.leftEncoder.distance-formerLeftDistance;
		formerLeftDistance = Drive.leftEncoder.distance;
		formerRightDistance = Drive.rightEncoder.distance;
		totalDistance = (currentLeftDistance + currentRightDistance)/2;
		positionX += totalDistance * Math.Sin(navx.GetYaw);
		positionY += totalDistance * Math.Cos(navx.GetYaw);
		System.out.println(positionX + ", " + positionY);

	}
	
}
