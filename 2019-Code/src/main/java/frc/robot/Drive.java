/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.networktables.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
     * The PID Controller for the left side drive train to maintain a given rate
     */
    private PIDController lRatePID;

    /**
     * The PID Controller for the right side drive train to maintain a given rate
     */
    private PIDController rRatePID;

    /**
     * True if driving should use the encoders, false if the encoders
     * should not be used
     */
    private boolean encoderDrive;

    /**
     * The time when the left encoder reported not moving with a motor current
     */
    private long lRateTime;

    /**
     * The time when the right encoder reported not moving with a motor current
     */
    private long rRateTime;

    /**
     * Whether or not the Limelight has adjusted
     */
    private enum Adjusted
    {
        NO,
        TURNING,
        MOVING_FORWARD,
        BACKING_UP
    }

    /**
     * Stores the value from the enum
     */
    private Adjusted adjusted;

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
        lEncoder.setPIDSourceType(PIDSourceType.kRate);
        rEncoder.setPIDSourceType(PIDSourceType.kRate);
        rEncoder.setReverseDirection(true);

        navx = new AHRS(Constants.Drive.NAVX_PORT);
        resetNavx();
       
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

        lRatePID = new PIDController(Constants.Drive.RATE_P, Constants.Drive.RATE_I, 
            Constants.Drive.RATE_D, Constants.Drive.RATE_F, lEncoder, 
            (double output)->
            {
                setLeft(output);
            });
        lRatePID.setAbsoluteTolerance(Constants.Drive.RATE_TOLERANCE);
        lRatePID.setInputRange(Constants.Drive.RATE_INPUT_MIN, Constants.Drive.RATE_INPUT_MAX);
        lRatePID.setOutputRange(Constants.Drive.RATE_OUTPUT_MIN, Constants.Drive.RATE_OUTPUT_MAX);
        lRatePID.setPIDSourceType(PIDSourceType.kRate);

        rRatePID = new PIDController(Constants.Drive.RATE_P, Constants.Drive.RATE_I, 
        Constants.Drive.RATE_D, Constants.Drive.RATE_F, rEncoder, 
            (double output)->
            {
                setRight(output);
            });
        rRatePID.setAbsoluteTolerance(Constants.Drive.RATE_TOLERANCE);
        rRatePID.setInputRange(Constants.Drive.RATE_INPUT_MIN, Constants.Drive.RATE_INPUT_MAX);
        rRatePID.setOutputRange(Constants.Drive.RATE_OUTPUT_MIN, Constants.Drive.RATE_OUTPUT_MAX);
        rRatePID.setPIDSourceType(PIDSourceType.kRate);

        encoderDrive = true;
        
        adjusted = Adjusted.NO;

        // new ValuePrinter(()-> 
        //     {
        //         // SmartDashboard.putNumber("Navx Yaw: ", navx.getYaw());
        //         // SmartDashboard.putNumber("Navx Angle: ", getNavxAngle());
        //         // SmartDashboard.putNumber("Left Encoder: ", lEncoder.getDistance());
        //         // SmartDashboard.putNumber("Right Encoder: ", rEncoder.getDistance());
        //         // SmartDashboard.putNumber("Limelight Area: ", limelight.getEntry("ta").getDouble(0));
        //         // SmartDashboard.putNumber("Limelight X: ", limelight.getEntry("tx").getDouble(0));
        //         // SmartDashboard.putNumber("Limelight skew", limelight.getEntry("ts").getDouble(0.0));
        //         // SmartDashboard.putNumber("Limelight y: ", limelight.getEntry("ty").getDouble(0.0));
        //         // SmartDashboard.putNumber("Calculated angle from target: ", Math.toDegrees(getAngleOffset()));

        //         SmartDashboard.putNumber("Left rate: ", lEncoder.getRate());
        //         SmartDashboard.putNumber("Right rate: ", rEncoder.getRate());
        //         SmartDashboard.putNumber("right setPoint: ", rRatePID.getSetpoint());
        //         SmartDashboard.putNumber("left setpoint: ", lRatePID.getSetpoint());
        //         // SmartDashboard.putNumber("offset Target", getAngleOffset());
        //     },
        //     ValuePrinter.HIGHEST_PRIORITY);
    }

    /**
     * Sets the power for the left side of the drive train.
     * @param power
     *  The power to set the left motors to from -1 to 1     
     */
    public void setLeft(double power)
    {
        lMotor1.set(ControlMode.PercentOutput, -power);
        lMotor2.set(ControlMode.PercentOutput, -power);
    }

    /**
     * Sets the power for the right side of the drive train.
     * @param power
     *  The power to set the right motors to from -1 to 1     
     */
    public void setRight(double power)
    {
        rMotor1.set(ControlMode.PercentOutput, power);
        rMotor2.set(ControlMode.PercentOutput, power);
    }
    
    /**
     * Drives the left side at the given rates using encoders with a PID Controller
     * or sets the motor power if the encoders are suspected to not be functioning.
     * @param power
     *  The power to drive the wheels at from -1 to 1
     * @param begin
     *  True to enable PID, false to only set the setpoint
     */
    public void driveEncoderLeft(double power, boolean begin)
    {
        if(encoderDrive) 
        {
            if(begin)
            {
                lRatePID.enable();
            }

            //Drive using encoders
            //Check motor current and wheel rates to deicde if encoders are functioning
            if(lMotor1.getOutputCurrent() > Constants.Drive.ENCODER_CURRENT_THRESHOLD && lEncoder.getRate() == 0 && lRatePID.getSetpoint() > 2)
            {
                if(lRateTime == 0)
                {
                    //Save time when encoder is first suspected broken
                    lRateTime = System.currentTimeMillis();
                    lRatePID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
                else if(System.currentTimeMillis() - lRateTime > Constants.Drive.ENCODER_TIMEOUT)
                {
                    //Drive by power if encoder is suspected to not be working
                    //Wheel rate is 0 while motor is getting a current for a 
                    //given amount of time
                    lRatePID.disable();
                    rRatePID.disable();
                    encoderDrive = false;
                    setLeft(power);
                    DriverStation.reportWarning("Encoders seem to not be working at the moment :(", false);
                }
                else
                {
                    //Set setpoint if everything is working for now
                    lRatePID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
            }
            else
            {
                //Sets PID at specified rate and resets encoder timeout
                lRateTime = 0; 
                lRatePID.setSetpoint(Constants.Drive.MAX_SPEED * power);
            }
        }
        else
        {
            //Set motor power if encoders are suspected to not be working
            setLeft(power);
        }
    }

    /**
     * Drives the right side at the given rates using encoders with a PID Controller
     * or sets the motor power if the encoders are suspected to not be functioning.
     * @param power
     *  The power to drive the wheels at from -1 to 1
     * @param begin
     *  True to enable PID, false to only set the setpoint
     */
    public void driveEncoderRight(double power, boolean begin)
    {
        if(encoderDrive)
        {
            if(begin)
            {
                rRatePID.enable();
            }

            //Drive using encoders
            //Check motor current and wheel rates to deicde if encoders are functioning
            if(rMotor1.getOutputCurrent() > Constants.Drive.ENCODER_CURRENT_THRESHOLD && rEncoder.getRate() == 0 && rRatePID.getSetpoint() > 2)
            {
                if(rRateTime == 0)
                {
                    //Save time when encoder is first suspected broken
                    rRateTime = System.currentTimeMillis();
                    rRatePID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
                else if(System.currentTimeMillis() - rRateTime > Constants.Drive.ENCODER_TIMEOUT)
                {
                    //Drive by power if encoder is suspected to not be working
                    //Wheel rate is 0 while motor is getting a current for a 
                    //given amount of time
                    lRatePID.disable();
                    rRatePID.disable();
                    encoderDrive = false;
                    setRight(power);
                    DriverStation.reportWarning("Encoders seem to not be working at the moment :(", false);
                }
                else
                {
                    //Set setpoint if everything is working for now
                    rRatePID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
            }
            else
            {
                //Sets PID at specified rate and resets encoder timeout
                rRateTime = 0;
                rRatePID.setSetpoint(Constants.Drive.MAX_SPEED * power);
            }
        }
        else
        {
            //Set motor power if encoders are suspected to not be working
            setRight(power);
        }
    }

    /**
     * Zeros the yaw and turns the robot the given amount of degrees.
     * @param degrees
     *  The amount of degrees to turn from -180 to 180
     */
    public void beginRelativeTurn(double degrees)
    {
        disableDriverAssist();
        zeroNavx();
        turnPID.setSetpoint(degrees);
        turnPID.enable();
    }

    /**
     * Rotates the robot to the specified angle relative to the robot's angle 
     * at the beginning of the match.
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
     * Makes the robot drive straight by using PID to control the speed the
     * wheels while using the Navx to correct for any drift.
     * @param power
     *  The multiplier for the max speed from -1 to 1 to set the set point to
     * @param begin
     *  True if the heading should be set to zero, and enable the PID, 
     *  false to align with the current zero
     */
    public void driveStraight(double power, boolean begin)
    {
        if(begin)
        {
            disableDriverAssist();
            zeroNavx();
        }
         
        double adjust = navx.getYaw() * 0.01;
        driveEncoderLeft(power - adjust, begin); 
        driveEncoderRight(power + adjust, begin);
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
     * a relative turn does not have an effect 
     * @return
     *  The degrees recorded by the Navx yaw
     */
    public double getNavxAngle()
    {
        return navx.getAngle();
    }

    /**
     * Zeros the Navx while setting the offset to maintain current 
     * position relative to the field.
     */
    private void zeroNavx()
    {
        navx.setAngleAdjustment(navx.getAngleAdjustment() + navx.getYaw());
        navx.zeroYaw();
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
     * Disables the PIDs and sets limelight to streaming mode.
     */
    public void disableDriverAssist()
    {
        turnPID.disable();
        lRatePID.disable();
        rRatePID.disable();
        lRatePID.setSetpoint(0.0);
        rRatePID.setSetpoint(0.0);
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("camMode").setNumber(1);
        adjusted = Adjusted.NO;
    }

    /**
     * Aligns the robot using the Lime Light.
     * @return
     *  True if the robot is aligned, false if still being aligned
     */
    public boolean limeLightAlign()
    {
        if(adjusted == Adjusted.NO && Math.abs(getAngleOffset()) > 12)
        {
            //Don't begin if robot is not lined up enough
            DriverStation.reportError("Not lined up enough for limelight", false);
            return false;
        }

        //Prepare for vision tracking
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("camMode").setNumber(0);
        
        if(limelight.getEntry("tv").getDouble(0.0) == 0) 
        {
            //Exit if no target is detected
            return false;
        }

        if(adjusted == Adjusted.BACKING_UP)
        {
            //Back up until far enough away
            // double offset = getAngleOffset();
            // if(Math.abs(offset) < 3)
            // {
            //     //Correct for tx offset
            //     double xOffset = limelight.getEntry("tx").getDouble(0.0);
            //     double p = 0.025;
            //     double adjust = p * xOffset;
    
            //     //Go faster on left to  
            //     setLeft(-0.4 - adjust);
            //     setRight(-0.4 + adjust);
            // }
            // else
            // {
            //     //Correct for angle offset
            //     double correction = getAngleOffset() * -0.01;
            //     setLeft(-0.4 - correction);
            //     setRight(-0.4 + correction);
            // }

            driveStraight(-0.5, false);

            if(limelight.getEntry("ty").getDouble(0) < 2)
            {
                //Stop backing up when far enough away
                adjusted = Adjusted.TURNING;
                lRatePID.disable();
                rRatePID.disable();
            }
        }
        else if(limelight.getEntry("ty").getDouble(0) > 13) 
        {
            //Robot is too close to keep going forward
            if(Math.abs(limelight.getEntry("tx").getDouble(0)) > 7)
            {
                //Back up if target is not centered with the target
                adjusted = Adjusted.BACKING_UP;
                driveStraight(-0.5, true);
            }
            else
            {
                //Finish if close and lined up
                disableDriverAssist();
                limelight.getEntry("ledMode").setDouble(2);
                return true;
            }
        }
        else if(adjusted == Adjusted.MOVING_FORWARD)
        {
            //Move towards target and correct if not centered
            double adjust = 0.025 * limelight.getEntry("tx").getDouble(0.0);
            setLeft(0.3 + adjust);
            setRight(0.3 - adjust);
        }
        else
        {
            //Turn to face the target
            double adjust;
            if(limelight.getEntry("tx").getDouble(0.0) >= 0)
            {
                adjust = Math.max(0.3, 0.04 * limelight.getEntry("tx").getDouble(0.0));
            }
            else
            {
                adjust = Math.min(-0.3, 0.04 * limelight.getEntry("tx").getDouble(0.0));
            }

            setLeft(adjust);
            setRight(-adjust);
            if(Math.abs(limelight.getEntry("tx").getDouble(0.0)) <3)
            {
                adjusted = Adjusted.MOVING_FORWARD;
            }
        }

        return false; 
    }

    /**
     * Returns the offset of robot angle to the nearest 45.
     * @return
     *  The offset of the robot
     */
    private double getAngleOffset()
    {
        int target = 45 * (int) Math.round(navx.getAngle() / 45);

        return target - navx.getAngle();
    }

    /**
     * Tests systems for drive.
     */
    public void systemTest()
    {
        setLeft(-0.3);
        setRight(0.3);

        Util.threadSleep(10);
    
    }
}