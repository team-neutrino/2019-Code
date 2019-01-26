package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public class Constants
{

    public static final double PID_P = 0.03;
    public static final double PID_I = 0.00;
    public static final double PID_D = 0.045;
    public static final double PID_TOLERANCE = 2;
    public static final int PIXY_INIT_AUTO = 100;
    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int LED_CONTROLLER_PORT = 3;
    public static final int LEFT_MOTOR_ONE_PORT = 0;
    public static final int LEFT_MOTOR_TWO_PORT = 1;
    public static final int RIGHT_MOTOR_ONE_PORT = 2;
    public static final int RIGHT_MOTOR_TWO_PORT = 3;
    public static final int PIXY_CLOCKRATE = 1000000;
    public static final int LEFT_ENCODER_PORT_ONE = 0;
    public static final int LEFT_ENCODER_PORT_TWO = 1;
    public static final int RIGHT_ENCODER_PORT_ONE = 2;
    public static final int RIGHT_ENCODER_PORT_TWO = 3;
    public static final Port NAVX_PORT = SPI.Port.kMXP;
    public static final Port PIXY_PORT = SPI.Port.kOnboardCS0;
    public static final int ENCODER_DISTANCE_PER_PULSE = 90/Math.PI;

}