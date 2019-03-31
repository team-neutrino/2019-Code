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
import edu.wpi.first.wpilibj.Ultrasonic;
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
     * The PID Controller for the left side drive train when driving straight
     */
    private PIDController leftStraightPID;

    /**
     * The PID Controller for the right side drive train when driving straight
     */
    private PIDController rightStraightPID;

    /**
     * True if the robot is backing up while using the limelight, false if still aligning forawrd
     */
    private boolean backingUp;

    /**
     * True if driving should use the encoders, false if the encoders
     * should not be used probably due to not functioning
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
     * The system's most recent time, in milliseconds, when the robot panel button has last been pressed. 
     */
    //private long startTime;

    /**
     * For the robot panel button, there will need to be some sort of button object for this to work.
     * I assumed that 1 meant pressed and 0 meant not pressed for this nonexistent button.
     */

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

        leftStraightPID = new PIDController(Constants.Drive.RATE_P, Constants.Drive.RATE_I, 
            Constants.Drive.RATE_D, lEncoder, 
            (double output)->
            {
                setLeft(output);
            });
        leftStraightPID.setAbsoluteTolerance(Constants.Drive.RATE_TOLERANCE);
        leftStraightPID.setInputRange(Constants.Drive.RATE_INPUT_MIN, Constants.Drive.RATE_INPUT_MAX);
        leftStraightPID.setOutputRange(Constants.Drive.RATE_OUTPUT_MIN, Constants.Drive.RATE_OUTPUT_MAX);

        rightStraightPID = new PIDController(Constants.Drive.RATE_P, Constants.Drive.RATE_I, 
        Constants.Drive.RATE_D, rEncoder, 
            (double output)->
            {
                setRight(output);
            });
        rightStraightPID.setAbsoluteTolerance(Constants.Drive.RATE_TOLERANCE);
        rightStraightPID.setInputRange(Constants.Drive.RATE_INPUT_MIN, Constants.Drive.RATE_INPUT_MAX);
        rightStraightPID.setOutputRange(Constants.Drive.RATE_OUTPUT_MIN, Constants.Drive.RATE_OUTPUT_MAX);

        encoderDrive = true;
        
        // new ValuePrinter(()-> 
        //     {
        //         SmartDashboard.putNumber("Navx Yaw: ", navx.getYaw());
        //         SmartDashboard.putNumber("Navx Angle: ", getNavxAngle());
        //         // SmartDashboard.putNumber("Ultrasonic: ", ultrasonic.getRangeInches());
        //         // SmartDashboard.putNumber("Left Encoder: ", lEncoder.getDistance());
        //         // SmartDashboard.putNumber("Right Encoder: ", rEncoder.getDistance());
        //         SmartDashboard.putNumber("Limelight Area: ", limelight.getEntry("ta").getDouble(0));
        //         SmartDashboard.putNumber("Limelight X: ", limelight.getEntry("tx").getDouble(0));
        //         SmartDashboard.putNumber("Limelight skew", limelight.getEntry("ts").getDouble(0.0));
        //         SmartDashboard.putNumber("Limelight y: ", limelight.getEntry("ty").getDouble(0.0));
        //         // SmartDashboard.putNumber("Calculated angle from target: ", Math.toDegrees(getAngleOffset()));

        //         SmartDashboard.putNumber("Left rate: ", lEncoder.getRate());
        //         SmartDashboard.putNumber("Right rate: ", rEncoder.getRate());
        //         SmartDashboard.putNumber("right setPoint: ", rightStraightPID.getSetpoint());
        //         SmartDashboard.putNumber("Left setPoint: ", leftStraightPID.getSetpoint());
        //         SmartDashboard.putNumber("left error", leftStraightPID.getError());
        //         SmartDashboard.putNumber("right error: ", rightStraightPID.getError());
        //         SmartDashboard.putNumber("l current", lMotor1.getOutputCurrent());
        //         SmartDashboard.putNumber("offset Target", getAngleOffset());
        //     },
        //     ValuePrinter.HIGHEST_PRIORITY);
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
     * Drives the left side at the given rates using encoders and a PID Controller
     * or sets the motor power if the encoders are suspected to not be functioning.
     * @param power
     *  The power to drive the wheels at from -1 to 1
     * @param begin
     *  True to enable PID, false to only set the setpoint
     */
    public void driveEncoderLeft(double power, boolean begin)
    {
        if(begin)
        {
            leftStraightPID.enable();
        }

        if(encoderDrive) 
        {
            //Drive using encoders
            //Check motor current and wheel rates to deicde if encoders are functioning
            if(lMotor1.getOutputCurrent() > Constants.Drive.ENCODER_CURRENT_THRESHOLD && lEncoder.getRate() == 0)
            {
                if(lRateTime == 0)
                {
                    //Save time when encoder is first suspected broken
                    lRateTime = System.currentTimeMillis();
                    leftStraightPID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
                else if(System.currentTimeMillis() - lRateTime > Constants.Drive.ENCODER_TIMEOUT)
                {
                    //Drive by power if encoder is suspected to not be working
                    //Wheel rate is 0 while motor is getting a current for a 
                    //given amount of time
                    leftStraightPID.disable();
                    rightStraightPID.disable();
                    encoderDrive = false;
                    setLeft(power);
                    DriverStation.reportWarning("Encoders seem to not be working at the moment :(", false);
                }
                else
                {
                    //Set setpoint if everything is working for now
                    leftStraightPID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
            }
            else
            {
                //Sets PID at specified rate and resets encoder timeout
                lRateTime = 0;
                leftStraightPID.setSetpoint(Constants.Drive.MAX_SPEED * power);
            }
        }
        else
        {
            //Set motor power if encoders are suspected to not be working
            setLeft(power);
        }
    }

    /**
     * Drives the right side at the given rates using encoders and a PID Controller
     * or sets the motor power if the encoders are suspected to not be functioning.
     * @param power
     *  The power to drive the wheels at from -1 to 1
     * @param begin
     *  True to enable PID, false to only set the setpoint
     */
    public void driveEncoderRight(double power, boolean begin)
    {
        if(begin)
        {
            rightStraightPID.enable();
        }

        if(encoderDrive)
        {
            //Drive using encoders
            //Check motor current and wheel rates to deicde if encoders are functioning
            if(rMotor1.getOutputCurrent() > Constants.Drive.ENCODER_CURRENT_THRESHOLD && rEncoder.getRate() == 0)
            {
                if(rRateTime == 0)
                {
                    //Save time when encoder is first suspected broken
                    rRateTime = System.currentTimeMillis();
                    rightStraightPID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
                else if(System.currentTimeMillis() - rRateTime > Constants.Drive.ENCODER_TIMEOUT)
                {
                    //Drive by power if encoder is suspected to not be working
                    //Wheel rate is 0 while motor is getting a current for a 
                    //given amount of time
                    leftStraightPID.disable();
                    rightStraightPID.disable();
                    encoderDrive = false;
                    setRight(power);
                    DriverStation.reportWarning("Encoders seem to not be working at the moment :(", false);
                }
                else
                {
                    //Set setpoint if everything is working for now
                    rightStraightPID.setSetpoint(Constants.Drive.MAX_SPEED * power);
                }
            }
            else
            {
                //Sets PID at specified rate and resets encoder timeout
                rRateTime = 0;
                rightStraightPID.setSetpoint(Constants.Drive.MAX_SPEED * power);
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
        zeroNavx();
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
     * Makes the robot drive straight by using PID to control the speed the
     * wheels while using the Navx  to correct for any drift.
     * \]
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
            zeroNavx();
            if(encoderDrive)
            {
                leftStraightPID.enable();
                rightStraightPID.enable();
            }
        }
         
        double adjust = navx.getYaw() * 0.01;
        
        driveEncoderLeft(power - adjust, false); 
        driveEncoderRight(power + adjust, false);
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
     * Disables the PIDs and sets limelight in streaming mode.
     */
    public void disableDriverAssist()
    {
        turnPID.disable();
        usPID.disable();
        leftStraightPID.disable();
        rightStraightPID.disable();
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("camMode").setNumber(1);
        backingUp = false;
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
            //Exit if no target is detected
            return false;
        }

        if(backingUp)
        {
            //Back up until far enough away
            double offset = getAngleOffset();
            if(Math.abs(offset) < 3)
            {
                //Correct for tx offset
                double xOffset = limelight.getEntry("tx").getDouble(0.0);
                double p = 0.025;
                double adjust = p * xOffset;
    
                //Go faster on left to  
                setLeft(-0.4 - adjust);
                setRight(-0.4 + adjust);
            }
            else
            {
                //Correct for angle offset
                double correction = getAngleOffset() * -0.01;
                setLeft(-0.4 - correction);
                setRight(-0.4 + correction);
            }

            if(limelight.getEntry("ty").getDouble(0) < -2)
            {
                backingUp = false;
            }
        }
        else if(limelight.getEntry("ty").getDouble(0) > 15) 
        {
            //Robot is too close to keep going forward
            if(Math.abs(limelight.getEntry("tx").getDouble(0)) > 7 || Math.abs(getAngleOffset()) > 7)
            {
                //Back up if target is not centered or not flat against the target
                backingUp = true;
            }
            else
            {
                //Finish if close and lined up
                disableDriverAssist();
                return true;
            }
        }
        else
        {
            //Start lining up with proportional correction amount to the amount offset of tx
            double offset = limelight.getEntry("tx").getDouble(0.0);
            double p = 0.025;// + 0.01 / limelight.getEntry("ta").getDouble(0.0);
            double adjust = p * offset;
            //Get amount of power to add/subtract
            // double diff = Math.min(offset * p, 0.5);
            // diff = Math.max(diff, 0.1);

            //Add left side subtract right to turn
            setLeft(0.3 + adjust);
            setRight(0.3 - adjust);
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

    // /**
    //  * Returns an estimated angle of the rotation of the robot to a target
    //  * using the distortion of the image.
    //  * @return
    //  *  An estimated angle of the roatation of the robot from the target
    //  */
    // private double getAngleOffset()
    // {
    //     double[] xs = limelight.getEntry("tcornx").getDoubleArray(new double[0]);
    //     double[] ys = limelight.getEntry("tcorny").getDoubleArray(new double[0]);
        
    //     // for(int i = 0; i < xs.length; i++)
    //     // {
    //     //     System.out.println("x: " + xs[i] + " y: " + ys[i]);
    //     // }
    //     if(xs.length >= 8 && ys.length >= 8) //Check if Limelight is sending data
    //     {
    //         //Calculate first angle from triangle with hypotenuse from point 1 to 7
    //         double xDist1 = xs[7] - xs[1];
    //         double yDist1 = ys[1] - ys[7];
    //         double angle1 = Math.tanh(yDist1 / xDist1);

    //         //Calculate first angle from triangle with hypotenuse from point 0 to 6
    //         double xDist2 = xs[0] - xs[6];
    //         double yDist2 = ys[0] - ys[6];
    //         double angle2 = Math.tanh(yDist2 / xDist2);
    //         //System.out.println("angle 1:" + angle1 + " \n angle2: " + angle2 + "\n diff: " + (angle1 - angle2));
    //         //Return calculated angle from experimetally generated equation
    //         return Math.PI / 2 - (-0.7861 * (angle1 - angle2) + 1.215);
    //     }

    //     return 0.0;
    // }

    /**
     * check to see if panelButton has been pressed down
     * @return  
     * A long value if it has been pressed, 0 otherwise
     */
    //private long checklongPress()
    //{
        //if (panelButton == 1) 
        //{
            //return System.currentTimeMillis();
        //}
        //else
        //{
            //return 0;
        //}
    }