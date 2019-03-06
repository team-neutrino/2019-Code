/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Odometry class for tracking the position of the robot using encoders 
 * and gyroscope.
 * 
 * @author Team Neutrino
 * 
 */
public class Odometry implements Runnable
{
	//TODO integrate with Lidar & make image/map
	
	/**
	 * X position of the robot
	 */
	private double positionX;

	/**
	 * Y position of the robot
	 */
	private double positionY;

	/**
	 *  Average of the distance traveled by the two wheels since the last time it was calculated
	 */
	private double totalDistance;

	/**
	 * Distance traveled by the left wheel, as found by the previous calculation
	 */
	private double formerLeftDistance;

	/**
	 * Distance traveled by the right wheel, as found by the previous calculation
	 */
	private double formerRightDistance;
	
	/**
	 * Distance traveled by the left wheel since the previous calculation
	 */
	private double currentLeftDistance;

	/**
	 * Distance traveled by the left wheel since the previous calculation
	 */
	private double currentRightDistance;

	/** 
	 * Drive class reference
	 */
	private Drive drive;

	/**
	 * Whether this is the first frame the program is running
	 */
	private boolean firstFrame;

	/**
	 * Constructor for the odometry to use the given drive class.
	 */
	public Odometry(Drive drive)
	{
		this.drive = drive;
		firstFrame = true;

        new Thread(this).start();
        
        new ValuePrinter(()->
			{
				SmartDashboard.putNumber("X Position: ", positionX);
				SmartDashboard.putNumber("Y Position: ", positionY);
				SmartDashboard.putNumber("Current Left Distance: ", currentLeftDistance);
				SmartDashboard.putNumber("Current Right Distance: ", currentRightDistance);
				SmartDashboard.putNumber("Former Left Distance: ", formerLeftDistance);
				SmartDashboard.putNumber("Former Right Distance: ", formerRightDistance);
			},
			ValuePrinter.HIGH_PRIORITY);   
	}

	/**
	 * Returns the X position of the robot
	 * @return 
	 * 	The X position of the robot
	 */
	public double getPositionX()
	{
		return positionX;
	}

	/**
	 * Returns the Y position of the robot
	 * @return 
	 * 	The Y position of the robot
	 */
	public double getPositionY()
	{
		return positionY;
	}

	/**
	 * Tests if the robot is within a set range of X and Y positions
	 * @param minX
	 * 	The minimum X position
	 * @param maxX
	 * 	The maximum X position
	 * @param minY
	 * 	The minimum Y position
	 * @param maxY
	 * 	The maximum Y position
	 */
	public boolean isInRange(double minX, double maxX, double minY, double maxY)
	{
		if(minX < positionX && maxX > positionX && minY < positionY && maxY > positionY)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
    @Override
    public void run() 
    {
		//TODO test
		while(true)
		{
			if(firstFrame)
			{
				drive.resetNavx();
				firstFrame = false;
			}
			currentRightDistance = drive.getRightDistance()-formerRightDistance;
			currentLeftDistance = drive.getLeftDistance()-formerLeftDistance;
			formerLeftDistance = drive.getLeftDistance();
			formerRightDistance = drive.getRightDistance();
			totalDistance = (currentLeftDistance + currentRightDistance)/2;
			positionX += totalDistance * Math.sin(Math.toRadians(drive.getNavxAngle()));
			positionY += totalDistance * Math.cos(Math.toRadians(drive.getNavxAngle()));
		}
	}
}