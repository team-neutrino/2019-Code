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
import frc.robot.CargoTransport.ArmPosition;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
     * The solenoid to make sure the climber doesn't trigger accidently
     */
    private Solenoid antiClimber;

    /**
     * Flag for the beginning of a driver assist
     * used to enable and disable PID
     */
    private boolean initDriverAssist;

    // /**
    //  * The odometry object
    //  */
    // private Odometry odometry;

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
    private boolean armOverride;

    /**
     * True if the panel button is being used, false if not
     */
    private boolean usePanelButton;

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
     * Stores whether the lidar is in use or not.
     */
    private boolean lidarInUse;

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
        antiClimber = new Solenoid(5);
        climber.set(false);
        antiClimber.set(true);
        
        usePanelButton = true;

        cam = CameraServer.getInstance().startAutomaticCapture("Wide angle", 0);
        cam.setFPS(15);
        cam.setResolution(160, 120);

        lidar = new LidarRaspberry(drive);
        lidarInUse = false;

        //TODO do stuff with odometry
        //odometry = new Odometry(drive);

        // //Makes camera image into black and white and sends to driver station.
        new Thread(()->
            {
                CvSink frontSink = CameraServer.getInstance().getVideo(cam);
                CvSource frontOutputStream = CameraServer.getInstance().putVideo("Wide BW", 160, 120);

                Mat source = new Mat();
                Mat output = new Mat();

                while(true)
                {
                    frontSink.grabFrame(source);
                    if(source.size().area() > 2)
                    {
                        Imgproc.cvtColor(source, output, Imgproc.COLOR_RGB2GRAY);
                        frontOutputStream.putFrame(output);                    
                    }
                }
            }).start();

        // new ValuePrinter(()->
        //     {
        //         SmartDashboard.putNumber("Left Joystick: ", lJoy.getY());
        //         SmartDashboard.putNumber("Right Joystick: ", rJoy.getY());
        //     }, 
        //     ValuePrinter.NORMAL_PRIORITY);        
    }

    /**
     * Called once before the sandstorm period.
     */
    @Override
    public void autonomousInit() 
    {
        drive.resetNavx();
        // lidar.enable();
        // lidarInUse = true;
        //drive.driveStraight(0.0, true);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() 
    {
       // drive.driveStraight(0.4, false);

        //Util.threadSleep(2);
         teleopPeriodic();
        // lidar.update();
    }

    @Override
    public void teleopInit()
    {
        lidar.disable();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() 
    {
        //Drivetrain control and driver assist
        if(lJoy.getRawButton(Constants.LJoy.LIMELIGHT_ALIGN_BUTTON) 
            || rJoy.getRawButton(Constants.RJoy.LIMELIGHT_ALIGN_BUTTON)) 
        {
            //Line up with bay and deliver panel
            initDriverAssist = false;
            if(!deliverDone && drive.limeLightAlign()) 
            {
                //Deploy panel if not already deployed and is lined up
                drive.disableDriverAssist();
                //Ram
                drive.driveEncoderLeft(1.0, true);
                drive.driveEncoderRight(1.0, true);
                Util.threadSleep(300);
                drive.driveEncoderLeft(0.0, false);
                drive.driveEncoderRight(0.0, false);
                drive.disableDriverAssist();

                panelTransport.setPanelHold(false);
                panelTransport.setPushersOut(true);
                Util.threadSleep(10);

                deliverDone = true;
            }
        } 
        else if(lJoy.getRawButton(Constants.LJoy.TURN_FIELD_BUTTON_0))
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(0);
                initDriverAssist= false;
            }
        }
        else if(lJoy.getRawButton(Constants.LJoy.TURN_FIELD_BUTTON_90))
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(90);
                initDriverAssist= false;
            }
        }
        else if(lJoy.getRawButton(Constants.LJoy.TURN_FIELD_BUTTON_180))
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(180);
                initDriverAssist= false;
            }
        }
        else if(lJoy.getRawButton(Constants.LJoy.TURN_FIELD_BUTTON_270))
        {
            if(initDriverAssist)
            {
                drive.rotateToAngle(270);
                initDriverAssist= false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.TURN_ROBOT_BUTTON_NEG90))
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(-90);
                initDriverAssist = false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.TURN_ROBOT_BUTTON_NEG45))
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(-45);
                initDriverAssist = false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.TURN_ROBOT_BUTTON_90))
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(90);
                initDriverAssist = false;
            }
        }
        else if(rJoy.getRawButton(Constants.RJoy.TURN_ROBOT_BUTTON_45))
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(45);
                initDriverAssist = false;
            }
        }
        else 
        { 
            //Control drive train using joysticks with a dead zone
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
            
            //Drive straight
            if(rJoy.getRawButton(Constants.RJoy.DRIVE_STRAIGHT_BUTTON))
            {
                if(initDriverAssist)
                {
                    drive.driveStraight(lPower, true);
                    initDriverAssist = false;
                }
                else
                {
                    drive.driveStraight(lPower, false);
                }
            }
            else if(lJoy.getRawButton(Constants.LJoy.DRIVE_STRAIGHT_BUTTON))
            {
                if(initDriverAssist)
                {
                    drive.driveStraight(rPower, true);
                    initDriverAssist = false;
                }
                else
                {
                    drive.driveStraight(rPower, false);
                }         
            }
            else
            {
                //Disable PIDs from driver assist and sets boolean flags for driver assist being done
                if(!initDriverAssist)
                {
                    drive.disableDriverAssist();
                    initDriverAssist = true;
                    deliverDone = false;
                }

                //Set powers equal to go straight if joysticks are close enough together
                //TODO see if Joel likes this
                if(Math.abs(rPower - lPower) < 0.05)
                {
                    rPower = lPower;
                }

                //Set motor power using joysticks
                drive.setRight(rPower);
                drive.setLeft(lPower);
            }
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
            cargoTransport.setRoller(-xBox.getRawAxis(Constants.XBox.OUTTAKE_CARGO_AXIS) * 0.8);
        }

        //Panel transport control
        //drive.checkLongPress();
        if(xBox.getRawAxis(Constants.XBox.OUTTAKE_PANEL_AXIS) > 0.5)
        {
            panelTransport.setPanelHold(false);
            Util.threadSleep(Constants.Robot.HOLD_PUSH_WAIT);
            panelTransport.setPushersOut(true);
            holdOverride = false;
        }
        //else if((System.currentTimeMillis()-drive.checkLongPress()) > 1500 && drive.checkLongPress() != 0)
        //{
            //panelTransport.setPanelHold(false);
            //Util.threadSleep(Constants.Robot.HOLD_PUSH_WAIT);
            //panelTransport.setPushersOut(true);
            //holdOverride = false;
        //}
        else
        {
            panelTransport.setPushersOut(false);

            if(panelTransport.getButton() && xBox.getRawButton(Constants.XBox.INTAKE_CARGO_BUTTON) && usePanelButton)
            {
                panelTransport.setPanelHold(true);
                holdOverride = true;
            }
            else
            {
                // if(!xBox.getRawButton(Constants.XBox.INTAKE_CARGO_BUTTON))
                // {
                //     holdOverride = false;
                // }

                if(!holdOverride)
                {
                    panelTransport.setPanelHold(!xBox.getRawButton(Constants.XBox.INTAKE_PANEL_BUTTON));
                }
            }
        }

        //Climb if match time is in last 30 seconds and button is pushed
        //or when 2 buttons are pushed in case match time is incorrect
        if((DriverStation.getInstance().getMatchTime() <= 20 && xBox.getRawButton(Constants.XBox.CLIMB_BUTTON))
            || (xBox.getRawButton(Constants.XBox.CLIMB_BUTTON) && xBox.getRawButton(Constants.XBox.CLIMB_OVERRIDE_BUTTON))
            || (lJoy.getRawButton(1) && lJoy.getRawButton(2) && rJoy.getRawButton(1) && rJoy.getRawButton(2)))
        {
            antiClimber.set(false);
            climber.set(true);
        }
        else
        {
            antiClimber.set(true);
            climber.set(false);
        }
       
        //Override Control
        if(xBox.getRawButton(Constants.XBox.TOGGLE_ARM_OVERRIDE_BUTTON))
        {  
            //Only toggle once per button push
            if(tempIsOverride)
            {
                //Toggle cargo arm PID
                cargoTransport.togglePID();
                armOverride =! armOverride;
                tempIsOverride = false;
            }
        }
        else if(xBox.getRawButton(Constants.XBox.TOGGLE_PANEL_OVERRIDE_BUTTON))
        {            
            //Only toggle once per button push
            if(tempIsOverride)
            {
                usePanelButton = !usePanelButton;
                tempIsOverride = false;
            }
        }
        else
        {
            //Allow for toggle after button is released
            tempIsOverride = true;
        }

        //Sets arm motor power during override mode
        if(armOverride)
        {
            cargoTransport.overrideArm(xBox.getRawAxis(Constants.XBox.ARM_OVERRIDE_AXIS));
        }

        Util.threadSleep(1);
    }

    @Override
    public void disabledInit()
    {
        if (lidarInUse) 
        {
            lidar.disable();
        }
    }
}