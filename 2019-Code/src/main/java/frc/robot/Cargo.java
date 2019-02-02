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
 * Add your docs here.
 */
public class Cargo implements PIDSource, PIDOutput, ValuePrinter {

    /**
     * Controls the intake/output of cargo
     */
    private TalonSRX intakeMotor;
    /**
     * Moves the arm
     */
    private TalonSRX armMotor;
    /**
     * Encoder attatched to the motor that moves the arm
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
     * Contructor for the cargo manipulator. Port numbers are not final.
     */
    public Cargo()
    {
        intakeMotor = new TalonSRX(6);
        armMotor = new TalonSRX(4);
        armEncoder = new AnalogPotentiometer(8);
        pid = new PIDController(0, 0, 0, this, this);
        pid.setAbsoluteTolerance(3);
        pid.setInputRange(0, 200);
        pid.setOutputRange(-1, 1);
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

    /**
     * Sets the power of the intake/output motor
     * @param power
     * The power to set
     */
    public void setIntake(double power)
    {
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void print(){
        SmartDashboard.putNumber("Arm Encoder Value", armEncoder.get());
    }

}
