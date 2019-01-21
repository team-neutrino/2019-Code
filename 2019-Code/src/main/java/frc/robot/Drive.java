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
import edu.wpi.first.wpilibj.SPI;
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
     * Does all setup tasks for the drive train.
     */
    public void setup()
    {
        lMotor1 = new TalonSRX(0);
        lMotor2 = new TalonSRX(1);
        rMotor1 = new TalonSRX(2);
        rMotor2 = new TalonSRX(3);

        navx = new AHRS(SPI.Port.kMXP);

        lEncoder = new Encoder(0, 1);
        rEncoder = new Encoder(2, 3);
        
        navx.zeroYaw();

        lEncoder.setDistancePerPulse(Math.PI/90);
        rEncoder.setDistancePerPulse(Math.PI/90);

        lEncoder.reset();
        rEncoder.reset();

        pixy = new PixyCam();

        pid = new PIDController(0.03, 0.0, 0.045, navx, this);
        pid.setInputRange(-180.0, 180.0);
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsoluteTolerance(2);
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
        rMotor1.set(ControlMode.PercentOutput, power);
        rMotor2.set(ControlMode.PercentOutput, power);
    }

    /**
     * Turns the robot the given amount of degrees.
     * @param degrees
     *  The amount of degrees to turn from -180 to 180
     */
    public void turnDegrees(int degrees)
    {
        pid.setSetpoint(degrees);
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
     *  The degrees recordede by the Navz yaw
     */
    public double getNavxYaw()
    {
        return navx.getYaw();
    }

    /**
     * Estimates the angle to turn the robot to deliver game pieces
     * using the white lines in front of the bays.
     */
<<<<<<< HEAD
    public void estimateAngle()
=======
    public static int estimateAngle()
>>>>>>> e7d1e30a1fb0f4ecfbe8da7ee361289a9c6165db
    {
        if(pixy.getWidth() != 0)
        {
            System.out.println("Width: " + pixy.getWidth() + " Height: " + pixy.getHeight());

            double angle = Math.tan((double) pixy.getHeight() / pixy.getWidth() );
            
            System.out.println("Angle: " + Math.toDegrees(angle));

            try
            {
                Thread.sleep(1000);
            }
            catch(InterruptedException e)
            {

            }
            return (int) angle;
        }

        return 0;
    }

    @Override
    public void pidWrite(double output)
    {
        setLeft(output); 
        setRight(-output);
    }

}