/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Class for the cargo arm abd roller intake
 */
public class CargoTransport implements PIDSource, PIDOutput
{
    /**
     * Controls the intake/output of cargo
     */
    private TalonSRX intakeMotor;
    
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
    private PIDController pid;
    
    /**
     * A the type returned by getPIDSourceType
     */
    private PIDSourceType type;

    /**
     * Contructor for the cargo manipulator.
     */
    public CargoTransport()
    {
        intakeMotor = new TalonSRX(6);
        armMotor = new TalonSRX(4);
        armEncoder = new AnalogPotentiometer(8);
        pid = new PIDController(0, 0, 0, this, this);
        pid.setAbsoluteTolerance(3);
        pid.setInputRange(0, 200);
        pid.setOutputRange(-1, 1);
        pid.enable();

        new ValuePrinter(()-> 
        {
            SmartDashboard.putNumber("Arm Encoder Value", armEncoder.get());
        }, 
        ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Sets the power of the intake/output motor
     * @param power
     * The power to set
     */
    public void setIntake(double power)
    {
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the position of the cargo arm.
     * @param angle
     *  The encoder angle to hold the arm at
     */
    public void setArmPosition(int angle)
    {
        pid.setSetpoint(angle);
    }

    @Override
    public PIDSourceType getPIDSourceType()
    {
        return type;
    }

    @Override
    public void setPIDSourceType(PIDSourceType sourceType)
    {
        type = sourceType;
    }

    @Override
    public double pidGet()
    {
        return armEncoder.get();
    }

    @Override
    public void pidWrite(double output)
    {
        armMotor.set(ControlMode.PercentOutput, output);
    }
}