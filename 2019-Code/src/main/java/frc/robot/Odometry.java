/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Odometry implements Runnable
{
    private static double positionX;
	private static double positionY;
	//Average of the distance traveled by the two wheels since the last time it was calculated
	private static double totalDistance = 0;
	//Distance traveled by the left wheel, as found by the previous calculation
	private static double formerLeftDistance = 0;
	//Distance traveled by the right wheel, as found by the previous calculation
    private static double formerRightDistance = 0;
	//Distance traveled by the left wheel since the previous calculation
	private static double currentLeftDistance = 0;
	//Distance traveled by the left wheel since the previous calculation
	private static double currentRightDistance = 0;

    @Override
    public void run() 
    {
    	currentRightDistance = Drive.getRightDistance()-formerRightDistance;
		currentLeftDistance = Drive.getLeftDistance()-formerLeftDistance;
		formerLeftDistance = Drive.getLeftDistance();
		formerRightDistance = Drive.getRightDistance();
		totalDistance = (currentLeftDistance + currentRightDistance)/2;
		positionX += totalDistance * Math.sin(Drive.getNavxYaw());
		positionY += totalDistance * Math.cos(Drive.getNavxYaw());
		System.out.println(positionX + ", " + positionY);
	}
}