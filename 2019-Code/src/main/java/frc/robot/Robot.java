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

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
     * Controlling the cargo transport stuff
     */
    private CargoTransport cargoTransport;

    /**
     * The panel transporter object
     */
    private PanelTransport panelTransport;

    /**
     * The Solenoid for the climber
     */
    private Solenoid climber;

    /**
     * Controller for the pixy cam
     */
    private PixyController pixy;

    /**
     * Flag for the beginning of a driver assist
     * used to enable a PID 
     */
    private boolean initDriverAssist;

    /**
     * The odometry object
     */
    private Odometry odometry;

    /**
     * Lights that flash to indicate stuff. Port number not permanent.
     */
    private LEDController dynamicLights;

    /**
     * A connection to the lidar
     */
    private LidarRaspberry lidar;

    /**
     * Whether the panel was delivered during the driver assist or not.
     */
    private boolean deliverDone;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() 
    {
        lJoy = new Joystick(Constants.Robot.LEFT_JOYSTICK_PORT);
        rJoy = new Joystick(Constants.Robot.RIGHT_JOYSTICK_PORT);
        xBox = new XboxController(Constants.Robot.XBOX_CONTROLLER_PORT);
      
        drive = new Drive();
        cargoTransport = new CargoTransport();
        panelTransport = new PanelTransport();
        climber = new Solenoid(Constants.Robot.CLIMBER_CHANNEL);
        //pixy =  new PixyController();

        //TODO put lights on robot
       // dynamicLights = new LEDController(72, Mode.OFF);

        //TODO do stuff with odometry
        //odometry = new Odometry(drive);

        UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture("front", 0);
        frontCam.setResolution(160, 120);
        frontCam.setFPS(15); 

        UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture("back", 1);
        backCam.setResolution(160, 120);
        backCam.setFPS(15);

        // //Makes camera image into black and white and sends to driver station.
        // new Thread(()->
        //     {
        //         UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture("front", 0);
        //         frontCam.setResolution(160, 120);
        //         frontCam.setFPS(15); 

        //         UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture("back", 1);
        //         backCam.setResolution(160, 120);
        //         backCam.setFPS(15);

        //         CvSink frontSink = CameraServer.getInstance().getVideo(frontCam);
        //         CvSource frontOutputStream = CameraServer.getInstance().putVideo("Front BW", 160, 120);

        //         CvSink backSink = CameraServer.getInstance().getVideo(backCam);
        //         CvSource backOutputStream = CameraServer.getInstance().putVideo("Back BW", 160, 120);

        //         Mat source = new Mat();
        //         Mat output = new Mat();

        //         while(true)
        //         {
        //             frontSink.grabFrame(source);
        //             if(source.size().area() > 2)
        //             {
        //                 Imgproc.cvtColor(source, output, Imgproc.COLOR_RGB2GRAY);
        //                 frontOutputStream.putFrame(output);
        //             }

        //             backSink.grabFrame(source);
        //             if(source.size().area() > 2)
        //             {
        //                 Imgproc.cvtColor(source, output, Imgproc.COLOR_RGB2GRAY);
        //                 backOutputStream.putFrame(output);
        //             }
        //         }
        //     }).start();

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
        //TODO constants left side bay line up and right side bay line up buttons
        if(lJoy.getRawButton(8) || rJoy.getRawButton(7)) //Line up with and deliver panel
        {
            int angle = pixy.estimateAngle();
            initDriverAssist = true;

            //
            if(angle > 10)
            {
                if(initDriverAssist)
                {
                    if(lJoy.getRawButton(8))
                    {
                        angle = -angle;
                    }
                    drive.beginTurn(angle);

                    initDriverAssist = false;
                }
            }
            else
            {
                if(!deliverDone && drive.limeLightAlign())
                {
                    drive.disablePID();
                    //Ram
                    //TODO tune power and time
                    drive.setLeft(0.25);
                    drive.setRight(0.25);
                    Util.threadSleep(100);
                    drive.setLeft(0.0);
                    drive.setRight(0.0);

                    panelTransport.setPanelHold(false);
                    panelTransport.setPushersOut(true);
                    Util.threadSleep(10);

                    deliverDone = true;
                }
            }
        }
        else if(lJoy.getRawButton(Constants.Robot.NEG_45_DEG_FIELD_BUTTON))
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(-45);
                initDriverAssist = false;
            }
        }
        else if(lJoy.getRawButton(Constants.Robot.POS_45_DEG_FIELD_BUTTON))
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(45);
                initDriverAssist = false;
            }
        }
        else if(lJoy.getRawButton(Constants.Robot.NEG_45_DEG_ROBOT_BUTTON))
        {
            if(initDriverAssist)
            {
                drive.beginTurn(-45);
                initDriverAssist = false;
            }
        }
        else if(lJoy.getRawButton(Constants.Robot.POS_45_DEG_ROBOT_BUTTON))
        {
            if(initDriverAssist)
            {
                drive.beginTurn(45);
                initDriverAssist = false;
            }
        }
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
                deliverDone = false;
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
            cargoTransport.setRoller(0.8);

        }
        else
        {
            cargoTransport.setRoller(-xBox.getRawAxis(Constants.XBox.OUTTAKE_CARGO_AXIS));
        }

        //Panel transport control
        if(xBox.getRawAxis(Constants.XBox.OUTTAKE_PANEL_AXIS) > 0.5)
        {
            panelTransport.setPanelHold(false);
            panelTransport.setPushersOut(true);
        }
        else
        {
            panelTransport.setPushersOut(false);
            panelTransport.setPanelHold(!xBox.getRawButton(Constants.XBox.INTAKE_PANEL_BUTTON));
        }

        //Climb if match time is in last 30 seconds and button is pushed
        //or when 2 buttons are pushed in case match time is incorrect
        if((DriverStation.getInstance().getMatchTime() <= 30 && xBox.getRawButton(Constants.XBox.CLIMB_BUTTON))
            || (xBox.getRawButton(Constants.XBox.CLIMB_BUTTON) && xBox.getRawButton(Constants.XBox.CLIMB_OVERRIDE_BUTTON)))
        {
            climber.set(true);
            dynamicLights.setMessage("-....--.-...-.---");
            dynamicLights.setMode(LEDController.Mode.MORSE);
        }

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