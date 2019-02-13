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
        ROCKET_BACK(0), SHIP_BACK(0), SHIP_FORWARD(0), INTAKE(0);

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
     * Contructor for the cargo manipulator.
     */
    public CargoTransport()
    {
        //TODO values + add to constants
        rollerMotor = new TalonSRX(Constants.CargoTransport.ARM_MOTOR_DEVICE_NUM);
        armMotor = new TalonSRX(Constants.CargoTransport.ARM_MOTOR_DEVICE_NUM);
        armEncoder = new AnalogPotentiometer(Constants.CargoTransport.ARM_ENCODER_CHANNEL);
        armPID = new PIDController(Constants.CargoTransport.ARM_P, Constants.CargoTransport.ARM_I, 
            Constants.CargoTransport.ARM_D, this, this);
        armPID.setAbsoluteTolerance(Constants.CargoTransport.ARM_PID_TOLERANCE);
        armPID.setInputRange(Constants.CargoTransport.ARM_MIN_INPUT, Constants.CargoTransport.ARM_MAX_INPUT);
        armPID.setOutputRange(-1, 1);
        armPID.enable();

        new ValuePrinter(()-> 
            {
                SmartDashboard.putNumber("Arm Encoder Value", armEncoder.get());
            }, 
            ValuePrinter.NORMAL_PRIORITY);
    }

    /**
     * Sets the power of the roller motor
     * @param power
     *  The power to set roller motor to
     */
    public void setIntake(double power)
    {
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