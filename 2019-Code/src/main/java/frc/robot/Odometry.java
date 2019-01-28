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
    private double positionX;
	private double positionY;
	//Average of the distance traveled by the two wheels since the last time it was calculated
	private double totalDistance = 0;
	//Distance traveled by the left wheel, as found by the previous calculation
	private double formerLeftDistance = 0;
	//Distance traveled by the right wheel, as found by the previous calculation
    private double formerRightDistance = 0;
	//Distance traveled by the left wheel since the previous calculation
	private double currentLeftDistance = 0;
	//Distance traveled by the left wheel since the previous calculation
	private double currentRightDistance = 0;
	//Drive class reference
	private Drive Drive;
	//Whether this is the first frame the program is running
	private boolean firstFrame = true;

	/**
	 * Constructor for the odometry to use the given drive class.
	 */
	public Odometry(Drive drive)
	{
		Drive = drive;
	}

    @Override
    public void run() 
    {
		if(firstFrame){
			Drive.resetNavx();
			firstFrame = false;
		}
    		currentRightDistance = Drive.getRightDistance()-formerRightDistance;
		currentLeftDistance = Drive.getLeftDistance()-formerLeftDistance;
		formerLeftDistance = Drive.getLeftDistance();
		formerRightDistance = Drive.getRightDistance();
		totalDistance = (currentLeftDistance + currentRightDistance)/2;
		positionX += totalDistance * Math.sin(Drive.getNavxAngle());
		positionY += totalDistance * Math.cos(Drive.getNavxAngle());
		System.out.println(positionX + ", " + positionY);
	}
}
