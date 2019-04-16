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

    /**
     * A connection to the lidar
     */
    private LidarRaspberry lidar;

    /**
     * Whether the panel was delivered during the driver assist or not
     */
    private boolean deliverDone;

    /**
     * True if the cargo arm control is overriden, false if not
     */
    private boolean armOverride;

    /**
     * True if override has been toggled, false if it has not been toggled
     */
    private boolean tempIsOverride;

    /**
     * Stops two second autonomous drive
     */
    private boolean stopAuton;

    /**
     * Constructor to set Watchdog timeout to 35 ms
     */
    // public Robot()
    // {
    //     super(0.035);
    // }

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
        antiClimber = new Solenoid(Constants.Robot.ANTI_CLIMBER_CHANNEL);
        climber.set(false);
        antiClimber.set(true);
        
        CameraServer.getInstance().startAutomaticCapture("Wide angle", 0);

        lidar = new LidarRaspberry(drive);

        stopAuton = true;

        //new Odometry(drive);
        
        //  new ValuePrinter(()->
        //      {
        //          SmartDashboard.putNumber("Left Joystick: ", lJoy.getY());
        //          SmartDashboard.putNumber("Right Joystick: ", rJoy.getY());
        //      }, 
        //      ValuePrinter.NORMAL_PRIORITY);        
    }

    /**
     * Called once before the sandstorm period.
     */
    @Override
    public void autonomousInit() 
    {
        antiClimber.set(true);
        climber.set(false);

        drive.resetNavx();
        lidar.enable();

        drive.driveEncoderLeft(0.0, true);
        drive.driveEncoderRight(0.0, true);
        
        //Begin drive straight autonomous portion
        if(Math.abs(lJoy.getY()) < 0.25 && Math.abs(rJoy.getY()) < 0.25)
        {
            new Thread(()->
                {
                    Util.threadSleep(2125);
                    stopAuton = true;
                }).start();

            drive.driveStraight(-1, true);
            initDriverAssist = false;
            stopAuton = false;
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() 
    {
        if(stopAuton)
        {
            //Manual control
            teleopPeriodic();
        }
        else
        {
            //Check to end autonomouns portion
            if(Math.abs(lJoy.getY()) > 0.25 || Math.abs(rJoy.getY()) < -0.25)
            {
                stopAuton = true;
            }

            drive.driveStraight(-1, false);

            Util.threadSleep(1);
        }

        lidar.update();
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
        if(lJoy.getRawButton(Constants.LJoy.LIMELIGHT_ALIGN_BUTTON) || rJoy.getRawButton(Constants.RJoy.LIMELIGHT_ALIGN_BUTTON)) 
        {
            //Line up with bay and deliver panel
            if(initDriverAssist)
            {
                //Make sure PIDs are turned off 
                drive.disableDriverAssist();
                initDriverAssist = false;
            }

            if(!deliverDone && drive.limeLightAlign()) 
            {
                //Stop lining up with limelight when done.
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
        else if(rJoy.getRawButton(Constants.RJoy.TURN_ROBOT_BUTTON_NEG45))
        {
            if(initDriverAssist)
            {
                drive.beginRelativeTurn(-45);
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
            //Square powers while maintaining negatives
            double rPower = -rJoy.getY();
            if(Math.abs(rPower) < Constants.RJoy.DEAD_ZONE)
            {
                rPower = 0.0;
            }
            else if(rPower > 0)
            {
                rPower *= rPower;
            }
            else
            {
                rPower *= -rPower;
            }
            double lPower = -lJoy.getY();
            if(Math.abs(lPower) < Constants.LJoy.DEAD_ZONE)
            {
                lPower = 0.0;
            }
            else if(lPower > 0)
            {
                lPower *= lPower;
            }
            else
            {
                lPower *= -lPower;
            }

            // Drive straight
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

                    drive.driveEncoderLeft(0.0, true);
                    drive.driveEncoderRight(0.0, true);
                }

                //Set powers equal to go straight if joysticks are close enough together
                if(Math.abs(rPower - lPower) < 0.05)
                {
                    double power = (rPower + lPower) / 2;
                    rPower = power;
                    lPower = power;
                }

                //Set motor power using joysticks
                // drive.setRight(rPower);
                // drive.setLeft(lPower);
            
                drive.driveEncoderLeft(lPower, false);
                drive.driveEncoderRight(rPower, false);
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
        panelTransport.setPanelHold(!xBox.getRawButton(Constants.XBox.PANEL_HOLDER_BUTTON));
        panelTransport.setPusherOut(xBox.getRawAxis(Constants.XBox.PANEL_PUSHER_AXIS) > 0.25);

        //Climb if match time is in last 30 seconds and button is pushed
        //or when 2 buttons are pushed in case match time is incorrect
        if((xBox.getRawButton(Constants.XBox.CLIMB_BUTTON) && DriverStation.getInstance().getMatchTime() <= 20)
            || (xBox.getRawButton(Constants.XBox.CLIMB_BUTTON) && xBox.getRawButton(Constants.XBox.CLIMB_OVERRIDE_BUTTON))
            || (lJoy.getRawButton(1) && lJoy.getRawButton(2) && rJoy.getRawButton(1) && rJoy.getRawButton(2)))
        {    
            antiClimber.set(false);
            climber.set(true);
        }
       
        //Override Control
        if(xBox.getRawButton(Constants.XBox.TOGGLE_ARM_OVERRIDE_BUTTON))
        {  
            //Only toggle once per button push
            if(tempIsOverride)
            {
                //Toggle cargo arm PID
                cargoTransport.togglePID();
                armOverride = !armOverride;
                tempIsOverride = false;
            }
        }
        else if(lJoy.getRawButton(Constants.LJoy.TOGGLE_ENCODER_DRIVE) || rJoy.getRawButton(Constants.RJoy.TOGGLE_ENCODER_DRIVE))
        {
            if(tempIsOverride)
            {
                drive.toggleEncoderDrive();
                tempIsOverride = false;
                initDriverAssist = false;
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

        if(lJoy.getRawButton(Constants.LJoy.RESET_NAVX_BUTTON) || rJoy.getRawButton(Constants.RJoy.RESET_NAVX_BUTTON))
        {
            drive.resetNavx();
        }

        Util.threadSleep(1);
    }

    @Override
    public void disabledInit()
    {
        lidar.disable();
    }

    @Override
    public void testInit()
    {
        panelTransport.systemTest();
    }
}