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
 * Class for the cargo arm and roller intake.
 * 
 * @author Team Neutrino
 * 
 */
public class CargoTransport implements PIDSource, PIDOutput
{
    /**
     * Enum for the arm positions.
     */
    public static enum ArmPosition
    {
        STORED(0), DELIVER(0), INTAKE(0);

        /**
         * The angle of the encoder for the arm at the given position.
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
    private PIDController armPID;

    /**
     * Contructor for the cargo manipulator.
     */
    public CargoTransport()
    {
        //TODO values + add to constants
        intakeMotor = new TalonSRX(6);
        armMotor = new TalonSRX(4);
        armEncoder = new AnalogPotentiometer(8);
        armPID = new PIDController(0, 0, 0, this, this);
        armPID.setAbsoluteTolerance(3);
        armPID.setInputRange(0, 200);
        armPID.setOutputRange(-1, 1);
        armPID.enable();

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
    public void setArmPosition(ArmPosition position)
    {
        armPID.setSetpoint(position.angle);
    }

    @Override
    public double pidGet()
    {
        //TODO encoder wraparound/negative
        return armEncoder.get();
    }

    @Override
    public void pidWrite(double output)
    {
        armMotor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public PIDSourceType getPIDSourceType()
    {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType sourceType) {}
}