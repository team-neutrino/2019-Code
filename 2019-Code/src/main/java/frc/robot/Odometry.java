/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Odometry implements Runnable
{
	//TODO java doc style comments /** */
	//TODO integrate with Lidar & make image/map
	
    private double positionX;
	private double positionY;
	//Average of the distance traveled by the two wheels since the last time it was calculated
	private double totalDistance;
	//Distance traveled by the left wheel, as found by the previous calculation
	private double formerLeftDistance;
	//Distance traveled by the right wheel, as found by the previous calculation
    private double formerRightDistance;
	//Distance traveled by the left wheel since the previous calculation
	private double currentLeftDistance;
	//Distance traveled by the left wheel since the previous calculation
	private double currentRightDistance;
	//Drive class reference
	private Drive drive;
	//Whether this is the first frame the program is running
	private boolean firstFrame = true;

	/**
	 * Constructor for the odometry to use the given drive class.
	 */
	public Odometry(Drive drive)
	{
		this.drive = drive;
        new Thread(this).start();
        
        new ValuePrinter(()->
        {
            SmartDashboard.putNumber("X", positionX);
            SmartDashboard.putNumber("Y", positionY);
            SmartDashboard.putNumber("leftDistance", currentLeftDistance);
            SmartDashboard.putNumber("rightDistance", currentRightDistance);
            SmartDashboard.putNumber("left distance ", formerLeftDistance);
            SmartDashboard.putNumber("real right ", formerRightDistance);
        },
        ValuePrinter.HIGH_PRIORITY);
	}

	//TODO make getter methods for position
	
    @Override
    public void run() 
    {
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