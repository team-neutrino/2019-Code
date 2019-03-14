/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class for the cargo arm and roller intake.
 * 
 * @author Team Neutrino
 * 
 */
public class CargoTransport implements PIDOutput
{
    /**
     * Enum for the arm positions.
     */
    public static enum ArmPosition
    {
        ROCKET_BACK(Constants.CargoTransport.ROCKET_BACK_ANGLE), 
        SHIP_BACK(Constants.CargoTransport.SHIP_BACK_ANGLE), 
        SHIP_FORWARD(Constants.CargoTransport.SHIP_FORWARD_ANGLE), 
        ARM_DOWN(Constants.CargoTransport.ARM_DOWN_ANGLE);

        /**
         * The angle of the encoder for the arm at the given position
         */
        private final int angle;

        /**
         * Makes arm position with the given encoder angle.
         * @param angle
         *  The angle the encoder will be at for this position
         */
        ArmPosition(int angle)
        {
            this.angle = angle;
        }
    }

    /**
     * Motor to control the intake/outtake of cargo transport
     */
    private TalonSRX rollerMotor;
    
    /**
     * Motor that controls the arm
     */
    private TalonSRX armMotor;
    
    /**
     * Encoder for the arm
     */
    private AnalogPotentiometer armEncoder;
    
    /**
     * Controls the arm position
     */
    private PIDController armPID;

    /**
     * The time when the motor went over the current threshold 
     * or 0 if the motor is not over current
     */
    private long currentOverTime;

    /**
     * Contructor for the cargo manipulator.
     */
    public CargoTransport()
    {
        rollerMotor = new TalonSRX(Constants.CargoTransport.ROLLER_MOTOR_DEVICE_NUM);
        armMotor = new TalonSRX(Constants.CargoTransport.ARM_MOTOR_DEVICE_NUM);
        
        armEncoder = new AnalogPotentiometer(Constants.CargoTransport.ARM_ENCODER_CHANNEL, Constants.CargoTransport.ENCODER_RANGE, 0);

        //TODO tune PID values when spring is attached
        armPID = new PIDController(Constants.CargoTransport.ARM_P, Constants.CargoTransport.ARM_I, 
            Constants.CargoTransport.ARM_D, armEncoder, this);
        armPID.setAbsoluteTolerance(Constants.CargoTransport.ARM_PID_TOLERANCE);
        armPID.setInputRange(Constants.CargoTransport.ARM_MIN_INPUT, Constants.CargoTransport.ARM_MAX_INPUT);
        armPID.setOutputRange(-Constants.CargoTransport.PID_OUTPUT_RANGE, Constants.CargoTransport.PID_OUTPUT_RANGE);
        setArmPosition(ArmPosition.SHIP_FORWARD);
        armPID.enable();

        currentOverTime = 0;

        new ValuePrinter(()-> 
            {
                SmartDashboard.putNumber("Arm Encoder Value", armEncoder.get());
                SmartDashboard.putNumber("Arm setpoint: ", armPID.getSetpoint());
            }, 
            ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Sets the power of the roller motor with current sensing so the motor does
     * not stall for more than 100 milliseconds.
     * @param power
     *  The power to set roller motor to, -1 out 1 in
     */
    public void setRoller(double power)
    {
        if(Math.abs(power) < 0.1)
		{
			power = 0.0;
			currentOverTime = 0;
		}
		else if(rollerMotor.getOutputCurrent() > Constants.CargoTransport.STALLED_CURRENT)
		{
            if(currentOverTime == 0)
            {
                currentOverTime = System.currentTimeMillis();
            }
            else if(System.currentTimeMillis() - currentOverTime > Constants.CargoTransport.HIGH_CURRENT_TIME_MAX)
            {
                power = 0.0;
            }
        }
        else
        {
            currentOverTime = 0;
        }
		
        rollerMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the position of the cargo arm.
     * @param position
     *  The position to hold the arm at
     */
    public void setArmPosition(ArmPosition position)
    {
        armPID.setSetpoint(position.angle);
    }

    /**
     * Turns the arm PID on if it is off and off if it is on 
     * to toggle override mode.
     */
    public void togglePID()
    {
        if(armPID.isEnabled())
        {
            armPID.disable();
        }
        else
        {
            armPID.enable();
        }
    }

    /**
     * Directly set arm motor power to the given value for override mode.
     * @param power
     *  The power to set the arm motor to form -1 to 1
     */
    public void overrideArm(double power)
    {
        armMotor.set(ControlMode.PercentOutput, -power);
    }  
    
    @Override
    public void pidWrite(double output)
    {
        //Limits motor power when gravity is assisting
        if(armEncoder.get() > Constants.CargoTransport.ARM_UP_ANGLE)
        {
            if(output < 0)
            {
                output *= Constants.CargoTransport.GRAVITY_ASSIST_MULTIPLIER;
            }
            armMotor.set(ControlMode.PercentOutput, output);
        }
        else
        {
            if(output > 0)
            {
                output *= Constants.CargoTransport.GRAVITY_ASSIST_MULTIPLIER;
            }
            armMotor.set(ControlMode.PercentOutput, output);
        }
    }  
}