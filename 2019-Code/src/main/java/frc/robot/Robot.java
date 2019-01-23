/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  /**
   * The left drive joystick
   */
  private Joystick lJoy;

  /**
   * The right drive joystick
   */
  private Joystick  rJoy;
  
  /**
   * A drive class object
   */
  private Drive Drive;

  private boolean initTurn;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    lJoy = new Joystick(0);
    rJoy = new Joystick(1);
    Drive = new Drive();
        
    new Thread(() -> 
    { 
      while(true)
      {
        SmartDashboard.putNumber("Left Joystick", lJoy.getY());
        SmartDashboard.putNumber("Right Joystick", rJoy.getY());
        Drive.print();

        try
        {
          Thread.sleep(500);
        }
        catch(InterruptedException e)
        {
          e.printStackTrace();
        }
      }
    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()  
  {

  }

  /**
   * Called once before the sandstorm period.
   */
  @Override
  public void autonomousInit() 
  {
    Drive.turnDegrees(90);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    if(lJoy.getRawButton(8) || rJoy.getRawButton(7)) //Turn using Pixy
    {
      if(initTurn)
      {
        int angle = Drive.estimateAngle();
        if(lJoy.getRawButton(8))
        {
          angle = -angle;
        }

        Drive.turnDegrees(angle);
        initTurn = false;
      }
    }
    else //Control drive train using joysticks with a dead zone
    {
      //Disable PIDs from driver assist
      if(!initTurn)
      {
        //TODO disable turn pid
        initTurn = true;
      }

      double rPower = -rJoy.getY();
      if(Math.abs(rPower) < 0.1)
      {
        rPower = 0.0;
      }
      Drive.setRight(rPower);

      double lPower = - lJoy.getY();
      if(Math.abs(lPower) < 0.1)
      {
        lPower = 0.0;
      }
      Drive.setLeft(lPower);
    }

    try
    {
      Thread.sleep(1);
    }
    catch(InterruptedException e)
    {
      e.printStackTrace();
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}
}