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
     * The navx IMU
     */
    private AHRS navx;

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
     * A PID object that makes the robot turn
     */
    private PIDController turnPID;      

    /**
     * PIDController for the Ultrasonic
     */
    private PIDController usPID;

    /**
     * Whether the robot is backing up after being too close to the target.
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

        navx = new AHRS(Constants.Drive.NAVX_PORT);
        ultrasonic = new Ultrasonic(Constants.Drive.ULTRASONIC_PORT_1, Constants.Drive.ULTRASONIC_PORT_2);
        lEncoder = new Encoder(Constants.Drive.LEFT_ENCODER_PORT_ONE, Constants.Drive.LEFT_ENCODER_PORT_TWO);
        rEncoder = new Encoder(Constants.Drive.RIGHT_ENCODER_PORT_ONE, Constants.Drive.RIGHT_ENCODER_PORT_TWO);

        lEncoder.setDistancePerPulse(Constants.Drive.ENCODER_DISTANCE_PER_PULSE);
        rEncoder.setDistancePerPulse(Constants.Drive.ENCODER_DISTANCE_PER_PULSE);

        turnPID = new PIDController(Constants.Drive.TURN_P, 
            Constants.Drive.TURN_I, Constants.Drive.TURN_D, navx,             
            (double output)->
            {
                if(turnPID.onTarget())
                {
                    setLeft(0);
                    setRight(0);
                }
                else
                {
                    setLeft(-output);
                    setRight(output);
                }
            });
        turnPID.setInputRange(-180.0, 180.0);
        turnPID.setOutputRange(-1.0, 1.0);
        turnPID.setAbsoluteTolerance(Constants.Drive.TURN_TOLERANCE);

        usPID = new PIDController(Constants.Drive.DISTANCE_P, Constants.Drive.DISTANCE_I, 
            Constants.Drive.DISTANCE_D, ultrasonic, 
            (double output)->
            {
                setLeft(output);
                setRight(output);
            });
        usPID.setAbsoluteTolerance(Constants.Drive.DISTANCE_TOLERANCE);
        usPID.setOutputRange(-1, 1);
        usPID.setInputRange(Constants.Drive.MIN_DISTANCE_RANGE, Constants.Drive.MAX_DISTANCE_RANGE);

        new ValuePrinter(()-> 
            {
                SmartDashboard.putNumber("Navx Yaw: ", navx.getYaw());
                SmartDashboard.putNumber("Navx Angle: ", navx.getAngle());
                SmartDashboard.putNumber("Ultrasonic: ", ultrasonic.getRangeInches());
                SmartDashboard.putNumber("Left Encoder: ", lEncoder.getDistance());
                SmartDashboard.putNumber("Right Encoder: ", rEncoder.getDistance());
            },
            ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Moves the robot forwards until it is a set distance away from the object in front of it
     * @param distance
     *  The distance from the object you want to reach (in inches)
     */
    public void moveToDistance(double distance)
    {
        usPID.enable();
        usPID.setSetpoint(distance);
    }

    /**
     * Sets the power for the left side of the drive train.
     * @param power
     *  The power to set the motor to from -1 to 1     
     */
    public void setLeft(double power)
    {
        lMotor1.set(ControlMode.PercentOutput, power);
        lMotor2.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the power for the right side of the drive train.
     * @param power
     *  The power to set the motor to from -1 to 1     
     */
    public void setRight(double power)
    {
        rMotor1.set(ControlMode.PercentOutput, -power);
        rMotor2.set(ControlMode.PercentOutput, -power);
    }

    /**
     * Zeros the yaw and turns the robot the given amount of degrees.
     * @param degrees
     *  The amount of degrees to turn from -180 to 180
     */
    public void beginTurn(double degrees)
    {
        navx.zeroYaw();
        turnPID.setSetpoint(degrees);
        turnPID.enable();
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
     * Returns the Navx yaw angle.
     * @return
     *  The degrees recorded by the Navx yaw
     */
    public double getNavxAngle()
    {
        return navx.getAngle();
    }

    /**
     * Resets the Navx.
     */
    public void resetNavx()
    {
        navx.reset();
    }

    public void zeroYaw()
    {
        navx.zeroYaw();
    }

    /**
     * Disables the PID threads.
     */
    public void disablePID()
    {
        turnPID.disable();
        usPID.disable();
    }

    /**
     * Rotates the robot to the specified angle (relative to field)
     * @param targetAngle
     *  The angle to turn to relative to robot at the start of the match
     */
    public void rotateToAngle(double targetAngle)
    {
        //TODO test negative/shortest turn
        double modAngle = navx.getAngle()%360;
        if(modAngle >= targetAngle)
        {
            if(modAngle-targetAngle <= (targetAngle+360)-modAngle)
            {
                beginTurn(modAngle-targetAngle);
            }
            else
            {
                beginTurn((targetAngle+360)-modAngle);
            }
        }
        else
        {
            if(targetAngle-modAngle <= (modAngle+360)-targetAngle)
            {
                beginTurn(targetAngle-modAngle);
            }
            else
            {
                beginTurn((modAngle+360)-targetAngle);
            }
        }
    }

    /**
     * Aligns the robot using the Lime Light.
     * @return
     *  True if the robot is aligned, false if still being aligned
     */
    public boolean limeLightAlign()
    {
        //TODO test
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) >= 50
        && Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) > 1)
        {
            setLeft(-1.0);
            setRight(-1.0);
            backingUp = true;
        }
        else if(backingUp)
        {
            setLeft(-1.0);
            setRight(-1.0);

            if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) < 25)
            {
                backingUp = false;
            }
        }
        else
        {
            if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) > 1)
            {
                setRight(0.85);
                setLeft(1);
            }
            else if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) < -1)
            {
                setLeft(0.85);
                setRight(1);
            }
            else if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) >= 75)
            {
                return true;
            }
            else
            {
                setLeft(1);
                setRight(1);
            }
        }

        return false;
    }
}