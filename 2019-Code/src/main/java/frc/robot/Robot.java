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
     * The drive object
     */
    private Drive drive;

    /**
     * Flag for the beginning of turn used to begin turn 
     * and disable PID when turn is finished
     */
    private boolean initTurn;

    /**
     * The controller for the white lights
     */
    private LEDController white;

    /**
     * The odometry object
     */
    private Odometry odometry;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() 
    {
        lJoy = new Joystick(Constants.LEFT_JOYSTICK_PORT);
        rJoy = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
        drive = new Drive();
        
        white = new LEDController(Constants.WHITE_LED_PORT, LEDController.Mode.ON);

        odometry = new Odometry(drive);

        //Thread for printing to Smart Dashboard
        new Thread(() -> 
        { 
            while(true)
            {
                SmartDashboard.putNumber("Left Joystick", lJoy.getY());
                SmartDashboard.putNumber("Right Joystick", rJoy.getY());
                drive.print();
                white.print();

                Util.threadSleep(500);
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
        drive.beginTurn(90);
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
                int angle = drive.estimateAngle();
                if(lJoy.getRawButton(8))
                {
                    angle = -angle;
                }

                initTurn = false;
                drive.beginTurn(angle);
            }

        }
        else //Control drive train using joysticks with a dead zone
        {
            //Disable PIDs from driver assist
            if(!initTurn)
            {
                drive.disablePID();
                initTurn = true;
            }

            double rPower = -rJoy.getY();
            if(Math.abs(rPower) < 0.1)
            {
                rPower = 0.0;
            }
            drive.setRight(rPower);

            double lPower = - lJoy.getY();
            if(Math.abs(lPower) < 0.1)
            {
                lPower = 0.0;
            }
            drive.setLeft(lPower);
        }

        Util.threadSleep(1);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}
}