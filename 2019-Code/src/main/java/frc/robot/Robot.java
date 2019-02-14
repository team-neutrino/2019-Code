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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CargoTransport.ArmPosition;
import edu.wpi.first.wpilibj.DriverStation;

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
     * The xBox controller
     */
    private XboxController xBox;
  
    /**
     * The drive object
     */
    private Drive drive;

    /**
     * The panel transporter object
     */
    private PanelTransport panelTransport;

    /**
     * Flag for the beginning of a driver assist
     * used to enable a PID 
     */
    private boolean initDriverAssist;

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
     * The number of lines the robot has passed (according to the PixyCam)
     */
    private int linesPassed;

    /**
     * Lights that flash to indicate stuff. Port number not permanent.
     */
    private LEDController dynamicLights;

    /**
     * A connection to the lidar
     */
    private LidarRaspberry lidar;

   /**
    * Controlling the cargo transport stuff
    */
    private CargoTransport cargoTransport;
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
        xBox = new XboxController(Constants.Robot.XBOX_CONTROLLER_PORT);
        drive = new Drive();
        climber = new Solenoid(Constants.Robot.CLIMBER_CHANNEL);
        panelTransport = new PanelTransport();
        cargoTransport = new CargoTransport();
        //TODO turn on only when needed
        white = new LEDController(Constants.Robot.WHITE_LED_PORT, LEDController.Mode.ON);
       // dynamicLights = new LEDController(72, Mode.OFF);

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
            if(initDriverAssist)
            {
                int angle = drive.estimateAngle();
                if(lJoy.getRawButton(8))
                {
                    angle = -angle;
                }

                initDriverAssist = false;
                drive.beginTurn(angle);
            }
        }
        //TODO turn pre-set degrees relative to robot/field
        else if(lJoy.getRawButton(Constants.LJoy.PREPARE_CLIMB_BUTTON) 
            || rJoy.getRawButton(Constants.RJoy.PREPARE_CLIMB_BUTTON))
        {
            if(initDriverAssist)
            {
                drive.moveToDistance(10);
            }
        }
        else //Control drive train using joysticks with a dead zone
        { 
            //Disable PIDs from driver assist
            if(!initDriverAssist)
            {
                drive.disablePID();
                initDriverAssist = true;
            }

            double rPower = -rJoy.getY();
            if(Math.abs(rPower) < 0.1)
            {
                rPower = 0.0;
            }
            drive.setRight(rPower);

            double lPower = -lJoy.getY();
            if(Math.abs(lPower) < 0.1)
            {
                lPower = 0.0;
            }
            drive.setLeft(lPower);
        }

        //Arm position control
        if(xBox.getRawButton(Constants.XBox.ARM_DOWN_BUTTON))
        {
            cargoTransport.setArmPosition(ArmPosition.ARM_DOWN);
        }
        else if(xBox.getRawButton(Constants.XBox.ROCKET_BACK_BUTTON))
        {
            cargoTransport.setArmPosition(ArmPosition.ROCKET_BACK);
        }
        else if(xBox.getRawButton(Constants.XBox.SHIP_BACK_BUTTON))
        {
            cargoTransport.setArmPosition(ArmPosition.SHIP_BACK);
        }
        else if(xBox.getRawButton(Constants.XBox.SHIP_FORWARD_BUTTON))
        {
            cargoTransport.setArmPosition(ArmPosition.SHIP_FORWARD);
        }

        //Roller speed control
        if(xBox.getRawButton(Constants.XBox.INTAKE_CARGO_BUTTON))
        {
            cargoTransport.setRoller(1.0);

        }
        else
        {
            cargoTransport.setRoller(xBox.getRawAxis(Constants.XBox.OUTTAKE_CARGO_AXIS));
        }


        //Panel transport control
        if(xBox.getRawAxis(Constants.XBox.OUTTAKE_PANEL_AXIS) > 0.5)
        {
            panelTransport.setPanelHold(false);
            panelTransport.setPushersOut(true);
        }
        else
        {
            panelTransport.setPanelHold(!xBox.getRawButton(Constants.XBox.INTAKE_PANEL_BUTTON));
        }

        //Climb if match time is in last 30 seconds and button is pushed
        //or when 2 buttons are pushed in case match time is incorrect
        if((station.getMatchTime() <= 30 && xBox.getRawButton(Constants.XBox.CLIMB_BUTTON))
            || (xBox.getRawButton(Constants.XBox.CLIMB_BUTTON) && xBox.getRawButton(Constants.XBox.CLIMB_OVERRIDE_BUTTON)))
        {
            climber.set(true);
            dynamicLights.setMessage("-....--.-...-.---");
            dynamicLights.setMode(LEDController.Mode.MORSE);
        }

        //TODO Change
        // if(pixy.isTracking())
        // {
        //     linesPassed++;
        //     dynamicLights.setFlashPulses(linesPassed);
        //     dynamicLights.setMode(Mode.FLASH);
        //     Util.threadSleep(1);
        //     dynamicLights.setMode(Mode.OFF);
        // }

        Util.threadSleep(1);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic(){}
}