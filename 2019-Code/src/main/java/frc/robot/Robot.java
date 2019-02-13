/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LEDController.Mode;

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
     * The Solenoid for the climber
     */
    private Solenoid climber;

    /**
     * An instance of the DriverStation class
     */
    private DriverStation station;

    /**
     * An instance of the PixyCam class
     */
    private PixyCam pixy;

    /**
     * The number of lines the robot has passed (according to the PixyCam)
     */
    private int linesPassed;

    /**
     * Lights that flash to indicate stuff. Port number not permanent.
     */
    private LEDController dynamicLights;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() 
    {
        //TODO values + add to constants
        lJoy = new Joystick(Constants.Robot.LEFT_JOYSTICK_PORT);
        rJoy = new Joystick(Constants.Robot.RIGHT_JOYSTICK_PORT);
        drive = new Drive();
        climber = new Solenoid(Constants.Robot.CLIMBER_CHANNEL);
        pixy = new PixyCam();

        //TODO turn on only when needed
        white = new LEDController(Constants.Robot.WHITE_LED_PORT, LEDController.Mode.ON);
        dynamicLights = new LEDController(72, Mode.OFF);

        //TODO do stuff with odometry
        odometry = new Odometry(drive);

        new ValuePrinter(()->
            {
                SmartDashboard.putNumber("Left Joystick: ", lJoy.getY());
                SmartDashboard.putNumber("Right Joystick: ", rJoy.getY());
            }, 
            ValuePrinter.NORMAL_PRIORITY);    
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
        teleopPeriodic();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() 
    {
        //Drivetrain control
        if(lJoy.getRawButton(8) || rJoy.getRawButton(7)) //Line up with bay TODO center, turn, limelight line-up, deliver
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
        //TODO turn pre-set degrees relative to robot/field
        //TODO climb set up
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

        //TODO cargo transport control

        //TODO hatch panel transport control

        //Climb if match time is in last 30 seconds and button is pushed
        //or when 2 buttons are pushed in case match time is incorrect
        if((station.getMatchTime() <= 30 && rJoy.getTriggerPressed())
            || (rJoy.getTriggerPressed() && lJoy.getTriggerPressed()))
        {
            drive.rotateToAngle(180);
            drive.moveToDistance(8);
            climber.set(true);
            dynamicLights.setMessage("-....--.-...-.---");
            dynamicLights.setMode(Mode.MORSE);
        }

        //TODO Change
        if(pixy.isTracking())
        {
            linesPassed++;
            dynamicLights.setFlashPulses(linesPassed);
            dynamicLights.setMode(Mode.FLASH);
            Util.threadSleep(1);
            dynamicLights.setMode(Mode.OFF);
        }

        Util.threadSleep(1);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}
}