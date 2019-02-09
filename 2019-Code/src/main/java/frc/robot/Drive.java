/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * The class for all drive components of the robot.
 * 
 * @author Team Neutrino
 * 
 */
public class Drive implements PIDOutput
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
     * The pixycam object
     */
    private PixyCam pixy;

    /**
     * A PID object that makes the robot turn
     */
    private PIDController pid;      

    /**
     * The navX angle mod 360
     */
    private double modAngle;

    /**
     * The Ultrasonic for measuring distance
     */
    private Ultrasonic ultrasonic;

    /**
     * PIDController for the Ultrasonic
     */
    private PIDController usPid;

    /**
     * Constructor for the drive train.
     */
    public Drive()
    {
        lMotor1 = new TalonSRX(Constants.LEFT_MOTOR_ONE_PORT);
        lMotor2 = new TalonSRX(Constants.LEFT_MOTOR_TWO_PORT);
        rMotor1 = new TalonSRX(Constants.RIGHT_MOTOR_ONE_PORT);
        rMotor2 = new TalonSRX(Constants.RIGHT_MOTOR_TWO_PORT);

        navx = new AHRS(Constants.NAVX_PORT);

        pixy = new PixyCam();

        lEncoder = new Encoder(Constants.LEFT_ENCODER_PORT_ONE, Constants.LEFT_ENCODER_PORT_TWO);
        rEncoder = new Encoder(Constants.RIGHT_ENCODER_PORT_ONE, Constants.RIGHT_ENCODER_PORT_TWO);

        lEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        rEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);

        pid = new PIDController(Constants.PID_P, Constants.PID_I, Constants.PID_D, navx, this);
        pid.setInputRange(-180.0, 180.0);
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsoluteTolerance(Constants.PID_TOLERANCE);

        ultrasonic = new Ultrasonic(5, 6);
        /*These numbers are not permanent.*/
        usPid = new PIDController(0.035, 0.00035, 0.01, ultrasonic, this);
        usPid.setAbsoluteTolerance(1);
        usPid.setOutputRange(-0.5,0.5);
        usPid.setInputRange(6, 200);

        new ValuePrinter(()-> 
        {
            SmartDashboard.putNumber("Navx: ", navx.getYaw());
            SmartDashboard.putNumber("Left Encoder: ", lEncoder.getDistance());
            SmartDashboard.putNumber("Right Encoder: ", rEncoder.getDistance());
            SmartDashboard.putNumber("Line Angle: ", estimateAngle());
        },
        ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Moves the robot forwards until it is a set distance away from whatever is in front of it
     * @param distance
     * The distance from the object you want to reach (in inches)
     */
    public void moveToDistance(double distance)
    {
        usPid.enable();
        usPid.setSetpoint(distance);
        while(!usPid.onTarget())
        {
            Util.threadSleep(1);
        }
        usPid.disable();
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
        pid.setSetpoint(degrees);
        pid.enable();
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
     * Returns the Navx yaw.
     * @return
     *  The degrees recorded by the Navz yaw
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

    /**
     * Disables the PID thread.
     */
    public void disablePID()
    {
        pid.disable();
    }

    /**
     * Estimates the angle to turn the robot to deliver game pieces
     * using the white lines in front of the bays.
     */
    public int estimateAngle()
    {
        if(pixy.isTracking())
        {
            double angle = Math.tanh((double) (pixy.getHeight()) / pixy.getWidth());

            return (int) Math.toDegrees(angle);
        }

        return 0;
    }

    /**
     * Rotates the robot to the specified angle (relative to field)
     */
    public void rotateToAngle(double targetAngle)
    {
        modAngle = navx.getAngle()%360;
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

    @Override
    public void pidWrite(double output)
    {
        setLeft(-output); 
        setRight(output);
    }
}