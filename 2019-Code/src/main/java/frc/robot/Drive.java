/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * The class for all drive components of the robot.
 * 
 * @author Team Neutrino
 * 
 */
public class Drive
{
    /**
     * The first motor controller for the left side drive train
     */
    private TalonSRX lMotor1;

    /**
     * The second motor controller for the left side drive train
     */
    private TalonSRX lMotor2;

    /**
     * The first motor controller for the right side drive train
     */
    private TalonSRX rMotor1;

    /**
     * The second motor controller for the right side the drive train
     */
    private TalonSRX rMotor2;

    /**
     * The encoder of the left side drive train
     */
    private Encoder lEncoder;

    /**
     * The encoder of the right side drive train
     */
    private Encoder rEncoder;

    /**
     * The Ultrasonic for measuring distance
     */
    private Ultrasonic ultrasonic;
    
    /**
     * The navx IMU
     */
    private AHRS navx;

    /**
     * The network table to access the Limelight data
     */
    private NetworkTable limelight;
    
    /**
     * A PID object that makes the robot turn
     */
    private PIDController turnPID;      

    /**
     * PIDController for using the Ultrasonic 
     * to drive a distance away from an object
     */
    private PIDController usPID;

    /**
     * True if the robot is backing up after being 
     * too close to the target, false otherwise
     */
    private boolean backingUp;

    /**
     * Constructor for the drive train.
     */
    public Drive()
    {
        lMotor1 = new TalonSRX(Constants.Drive.LEFT_MOTOR_ONE_PORT);
        lMotor2 = new TalonSRX(Constants.Drive.LEFT_MOTOR_TWO_PORT);
        rMotor1 = new TalonSRX(Constants.Drive.RIGHT_MOTOR_ONE_PORT);
        rMotor2 = new TalonSRX(Constants.Drive.RIGHT_MOTOR_TWO_PORT);

        lEncoder = new Encoder(Constants.Drive.LEFT_ENCODER_PORT_ONE, Constants.Drive.LEFT_ENCODER_PORT_TWO);
        rEncoder = new Encoder(Constants.Drive.RIGHT_ENCODER_PORT_ONE, Constants.Drive.RIGHT_ENCODER_PORT_TWO);
        lEncoder.setDistancePerPulse(Constants.Drive.ENCODER_DISTANCE_PER_PULSE);
        rEncoder.setDistancePerPulse(Constants.Drive.ENCODER_DISTANCE_PER_PULSE);

        navx = new AHRS(Constants.Drive.NAVX_PORT);
        navx.reset();

        ultrasonic = new Ultrasonic(Constants.Drive.ULTRASONIC_PORT_1, Constants.Drive.ULTRASONIC_PORT_2);
       
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("camMode").setNumber(1);

        turnPID = new PIDController(Constants.Drive.TURN_P, 
            Constants.Drive.TURN_I, Constants.Drive.TURN_D, navx,             
            (double output)->
            {
                setLeft(output);
                setRight(-output);
            });
        turnPID.setInputRange(Constants.Drive.TURN_INPUT_MIN, Constants.Drive.TURN_INPUT_MAX);
        turnPID.setOutputRange(Constants.Drive.TURN_OUTPUT_MIN, Constants.Drive.TURN_OUTPUT_MAX);
        turnPID.setAbsoluteTolerance(Constants.Drive.TURN_TOLERANCE);

        usPID = new PIDController(Constants.Drive.DISTANCE_P, Constants.Drive.DISTANCE_I, 
            Constants.Drive.DISTANCE_D, ultrasonic, 
            (double output)->
            {
                setLeft(-output);
                setRight(-output);
            });
        usPID.setAbsoluteTolerance(Constants.Drive.DISTANCE_TOLERANCE);
        usPID.setInputRange(Constants.Drive.DISTANCE_INPUT_MIN, Constants.Drive.DISTANCE_INPUT_MAX);
        usPID.setOutputRange(Constants.Drive.DISTANCE_OUTPUT_MIN, Constants.Drive.DISTANCE_OUTPUT_MAX);

        new ValuePrinter(()-> 
            {
                SmartDashboard.putNumber("Navx Yaw: ", navx.getYaw());
                SmartDashboard.putNumber("Navx Angle: ", getNavxAngle());
                SmartDashboard.putNumber("Ultrasonic: ", ultrasonic.getRangeInches());
                SmartDashboard.putNumber("Left Encoder: ", lEncoder.getDistance());
                SmartDashboard.putNumber("Right Encoder: ", rEncoder.getDistance());
                SmartDashboard.putNumber("Limelight Area: ", limelight.getEntry("ta").getDouble(0));
                SmartDashboard.putNumber("Limelight X: ", limelight.getEntry("tx").getDouble(0));
            },
            ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Sets the power for the left side of the drive train.
     * @param power
     *  The power to set the motor to from -1 to 1     
     */
    public void setLeft(double power)
    {
        lMotor1.set(ControlMode.PercentOutput, -power);
        lMotor2.set(ControlMode.PercentOutput, -power);
    }

    /**
     * Sets the power for the right side of the drive train.
     * @param power
     *  The power to set the motor to from -1 to 1     
     */
    public void setRight(double power)
    {
        rMotor1.set(ControlMode.PercentOutput, power);
        rMotor2.set(ControlMode.PercentOutput, power);
    }
    
    /**
     * Zeros the yaw and turns the robot the given amount of degrees.
     * @param degrees
     *  The amount of degrees to turn from -180 to 180
     */
    public void beginRelativeTurn(double degrees)
    {
        //Sets angle adjustment to keep robot angle relative to the field
        navx.setAngleAdjustment(navx.getAngleAdjustment() + navx.getYaw());

        navx.zeroYaw();
        turnPID.setSetpoint(degrees);
        turnPID.enable();
    }

    /**
     * Rotates the robot to the specified angle (relative to field)
     * @param targetAngle
     *  The angle to turn to relative to robot at the start of the match
     */
    public void rotateToAngle(double targetAngle)
    {
        //Get current position from 0 to 360
        double modAngle = getNavxAngle() % 360;
        if(modAngle < 0)
        {
            modAngle += 360;
        }

        //Amount needed to turn  to get to position from 0 to 360
        double turnAngle = targetAngle - modAngle;
        
        //Turn shortest distane from -180 to 180
        if(Math.abs(turnAngle) > 180)
        {
            if(turnAngle > 0)
            {
                turnAngle -= 360;
            }
            else
            {
                turnAngle += 360;
            }
        }

        beginRelativeTurn(turnAngle);
    }

    /**
     * Moves the robot forwards until it is a set distance away from the object in front of it
     * @param distance
     *  The distance from the object you want to reach (in inches)
     */
    public void moveToDistance(double distance)
    {
        usPID.setSetpoint(distance);
        usPID.enable();
    }

    /**
     * Returns the distance travelled recoreded by the left encoder.
     * @return
     *  The distance travelled in inches
     */
    public double getLeftDistance()
    {
        return lEncoder.getDistance();
    }

     /**
     * Returns the distance travelled recoreded by the right encoder.
     * @return
     *  The distance travelled in inches
     */
    public double getRightDistance()
    {
        return rEncoder.getDistance();
    }

    /**
     * Returns the continuous yaw angle from the navx. Zeroing the yaw during 
     * a relative turn does not have an effect on this.
     * @return
     *  The degrees recorded by the Navx yaw
     */
    public double getNavxAngle()
    {
        return navx.getAngle();
    }

    /**
     * Sets angle adjustment to 0 and sets the navx to 0.
     */
    public void resetNavx()
    {
        navx.setAngleAdjustment(0.0);
        navx.zeroYaw();
    }

    /**
     * Disables the PID threads and sets limelight in streaming mode.
     */
    public void disableDriverAssist()
    {
        turnPID.disable();
        usPID.disable();
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("camMode").setNumber(1);
    }

    /**
     * Aligns the robot using the Lime Light.
     * @return
     *  True if the robot is aligned, false if still being aligned
     */
    public boolean limeLightAlign()
    {
        turnPID.disable();
        usPID.disable();
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("camMode").setNumber(0);

        if(limelight.getEntry("tv").getDouble(0.0) == 0)
        {
            return false;
        }

        if(limelight.getEntry("ta").getDouble(0) > 7 && Math.abs(limelight.getEntry("tx").getDouble(0)) > 4)
        {
            //start backing up if too close and not aligned
            setLeft(-0.3);
            setRight(-0.3);
            backingUp = true;
        }
        else if(backingUp)
        {
            //Back up until far enough away
            setLeft(-0.3);
            setRight(-0.3);

            if(limelight.getEntry("ta").getDouble(0) < 2)
            {
                backingUp = false;
            }
        }
        else if(limelight.getEntry("ta").getDouble(0) >= 9)
        {
            //Lined up 
            return true;
        }
        else
        {
            //Start lining up with proportion of the x
            double offset = limelight.getEntry("tx").getDouble(0.0);
            double p = 0.025 + 0.01 / limelight.getEntry("ta").getDouble(0.0);

            //Get amount of power to add/subtract
            double diff = Math.min(offset * p, 0.5);
            diff = Math.max(diff, 0.1);

            //Add left side subtract right to turn
            setLeft(0.5 + diff);
            setRight(0.5 - diff);
            //TODO do actual math to get better propotional turn
            // if(limelight.getEntry("tx").getDouble(0) > 1)
            // {
            //     setRight(.15);
            //     double pow = 0.15 + limelight.getEntry("tx").getDouble(0.0) * 0.025;
            //     setLeft(pow);
            // }
            // else if(limelight.getEntry("tx").getDouble(0) < -1)
            // {
            //     setLeft(0.15);
            //     double pow = 0.15 + Math.abs(limelight.getEntry("tx").getDouble(0.0)) * 0.025;
            //     setRight(pow);
            // }
        }

        return false;
    }
}