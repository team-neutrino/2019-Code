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
    public static Joystick lJoy;

    /**
     * The right drive joystick
     */
    private Joystick  rJoy;

    /**
     * The xBox controller
     */
    private XboxController xBox;
  
    /**
     * The drive object for all drive train related functions
     */
    private Drive drive;

    /**
     * The cargo transport object for all cargo related functions
     */
    private CargoTransport cargoTransport;

    /**
     * The panel transport object for all panel functions
     */
    private PanelTransport panelTransport;

    /**
     * The Solenoid for the climber
     */
    private Solenoid climber;

    /**
     * Flag for the beginning of a driver assist
     * used to enable and disable PID
     */
    private boolean initDriverAssist;

    /**
     * The odometry object
     */
    private Odometry odometry;

    /**
     * A connection to the lidar
     */
    private LidarRaspberry lidar;

    /**
     * Whether the panel was delivered during the driver assist or not
     */
    private boolean deliverDone;

    /**
     * The USB camera for driver vision 
     */
    private UsbCamera cam;

    /**
     * True if the cargo arm control is overriden, false if not
     */
    private boolean isOverride;

    /**
     * True if override has been toggled, false if it has not been toggled
     */
    private boolean tempIsOverride;

    /**
     * True if the panel holder is in hold override and not to be put down, false to 
     * have complete control from the button monkey
     */
    private boolean holdOverride;
    
    /**
     * This shuts off the lidar once we get into teleop.
     */
    private boolean lidarSwitch;

    /**
     * This variable stores weather the lidar is in use or not.
     */
    private boolean lidarInUse;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() 
    {
        lidarInUse = false;
        lJoy = new Joystick(Constants.Robot.LEFT_JOYSTICK_PORT);
        rJoy = new Joystick(Constants.Robot.RIGHT_JOYSTICK_PORT);
        xBox = new XboxController(Constants.Robot.XBOX_CONTROLLER_PORT);
      
        drive = new Drive();
        cargoTransport = new CargoTransport();
        panelTransport = new PanelTransport();
        climber = new Solenoid(Constants.Robot.CLIMBER_CHANNEL);

        //TODO actually set camera resolution and frame rate
        cam = CameraServer.getInstance().startAutomaticCapture("Wide angle", 0);
        cam.setFPS(15);
        cam.setResolution(320, 240);

        lidarSwitch = true;

        //TODO do stuff with odometry
        //odometry = new Odometry(drive);

        // UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture("front", 0);
        // frontCam.setResolution(160, 120);
        // frontCam.setFPS(15); 

        // UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture("back", 1);
        // backCam.setResolution(160, 120);
        // backCam.setFPS(15);

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
        lidar = new LidarRaspberry(drive);
        
    }

    /**
     * Called once before the sandstorm period.
     */
    @Override
    public void autonomousInit() {
        lidar.enable();
        lidarInUse = true;
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
        //Turn off the lidar once we get out of auto.
        if(!isAutonomous() && lidarSwitch)
        {
            System.out.println("Out of auto!");
            lidarSwitch = false;
            lidar.disable();
        }
        lidar.update();

        //Drivetrain control and driver assist
        if(lJoy.getRawButton(Constants.LJoy.DELIVER_LEFT_SIDE_BUTTON) 
            || lJoy.getRawButton(Constants.LJoy.DELIVER_RIGHT_SIDE_BUTTON)) //Line up with bay and deliver panel
        {
            initDriverAssist = false;
            if(!deliverDone && drive.limeLightAlign()) //Deploy panel if not already deployed and is lined up
            {
                //drive.disableDriverAssist();
                //Ram
                //TODO tune power and time
                // drive.setLeft(0.25);
                // drive.setRight(0.25);
                // Util.threadSleep(100);
                // drive.setLeft(0.0);
                // drive.setRight(0.0);

                // panelTransport.setPanelHold(false);
                // panelTransport.setPushersOut(true);
                // Util.threadSleep(10);

                deliverDone = true;
            }
        } 
        else if(rJoy.getRawButton(Constants.RJoy.TURN_FIELD_BUTTON_270)) //Turn 270 relative to the field from robot init
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(270);
                initDriverAssist = false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.TURN_FIELD_BUTTON_90)) //Turn 90 relative to the field from robot init
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(90);
                initDriverAssist = false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.NEG_45_DEG_ROBOT_BUTTON))//Turn 45 relative to the robot
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(-45);
                initDriverAssist = false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.POS_45_DEG_ROBOT_BUTTON)) //Turn -45 relative to the robot
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(45);
                initDriverAssist = false;
            }
        }
        else if(lJoy.getRawButton(Constants.LJoy.PREPARE_CLIMB_BUTTON) //Drive to distance away from wall to set up climb
            || rJoy.getRawButton(Constants.RJoy.PREPARE_CLIMB_BUTTON)) //TODO might not need this climb set up
        {
            if(initDriverAssist)
            {
                drive.moveToDistance(Constants.Robot.CLIMB_DISTANCE);
                initDriverAssist = false;
            }
        }
        else //Control drive train using joysticks with a dead zone
        { 
            //Disable PIDs from driver assist and sets boolean flags for driver assist being done
            if(!initDriverAssist)
            {
                drive.disableDriverAssist();
                initDriverAssist = true;
                deliverDone = false;
            }

            //Get joystick values and make correct direction and with a dead zone
            double rPower = -rJoy.getY();
            if(Math.abs(rPower) < Constants.RJoy.DEAD_ZONE)
            {
                rPower = 0.0;
            }
            double lPower = -lJoy.getY();
            if(Math.abs(lPower) < Constants.LJoy.DEAD_ZONE)
            {
                lPower = 0.0;
            }
            
            //Drive straight by setting powers equal and compensating for robot mechanical drift
            //TODO make more consistent way to go straight
            if(rJoy.getRawButton(Constants.RJoy.DRIVE_STRAIGHT_BUTTON))
            {
                lPower = rPower * (lJoy.getZ()  + 1);
            }
            else if(lJoy.getRawButton(Constants.LJoy.DRIVE_STRAIGHT_BUTTON))
            {
                rPower = lPower;
            }

            drive.setRight(rPower);
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
            //TODO see if adding wait inbetween improves panel delivery
            panelTransport.setPanelHold(false);
            Util.threadSleep(20);
            panelTransport.setPushersOut(true);
            holdOverride = false;
        }
        else
        {
            panelTransport.setPushersOut(false);

            if(panelTransport.getButton())
            {
                panelTransport.setPanelHold(true);
                holdOverride = true;
            }
            else
            {
                if(!holdOverride)
                {
                    panelTransport.setPanelHold(!xBox.getRawButton(Constants.XBox.INTAKE_PANEL_BUTTON));
                }
            }
        }

        //Climb if match time is in last 30 seconds and button is pushed
        //or when 2 buttons are pushed in case match time is incorrect
        if((DriverStation.getInstance().getMatchTime() <= 30 && xBox.getRawButton(Constants.XBox.CLIMB_BUTTON))
            || (xBox.getRawButton(Constants.XBox.CLIMB_BUTTON) && xBox.getRawButton(Constants.XBox.CLIMB_OVERRIDE_BUTTON)))
        {
            climber.set(true);
        }
       
        //Toggle cargo arm override control
        if(xBox.getRawButton(9))
        {  
            if(tempIsOverride) //Only toggle once per button push
            {
                cargoTransport.togglePID();
                isOverride =! isOverride;
                tempIsOverride = false;
            }
        }
        else //Allow for toggle after button is released
        {
            tempIsOverride = true;
        }

        //Sets arm motor power during override mode
        if(isOverride)
        {
            cargoTransport.overrideArm(xBox.getRawAxis(1));
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
    public void robotPeriodic() {}

    @Override
    public void disabledInit()
    {
        if (lidarInUse) {
            try {
                lidar.disable();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}