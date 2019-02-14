package frc.robot;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.SerialPort;
/**
 * Constants for the robot
 * 
 * @author Team Neutrino
 * 
 */
public class Constants
{
    /**
     * Constants for the robot class.
     */
    public static class Robot
    {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int WHITE_LED_PORT = 3;
        public static final int XBOX_CONTROLLER_PORT = 0;

        public static final int CLIMBER_CHANNEL = 9;
    }

    /**
     * Constants for the X-Box controller button functions.
     */
    public static class XBox
    {
        public static final int ROCKET_BACK_BUTTON = 0;
        public static final int SHIP_BACK_BUTTON = 0;
        public static final int SHIP_FORWARD_BUTTON = 0;
        public static final int ARM_DOWN_BUTTON = 0;
        public static final int INTAKE_CARGO_BUTTON = 0;
        public static final int OUTTAKE_CARGO_BUTTON = 0;

        public static final int INTAKE_PANEL_BUTTON = 0;
        public static final int OUTTAKE_PANEL_BUTTON = 0;

        public static final int CLIMB_BUTTON = 0;
        public static final int CLIMB_OVERRIDE_BUTTON = 0;
    }

    /**
     * Constants for the left joystick button functions.
     */
    public static class LJoy
    {
        public static final int DELIVER_LEFT_SIDE_BUTTON = 0;
        public static final int DELIVER_RIGHT_SIDE_BUTTON = 0;

        public static final int PREPARE_CLIMB_BUTTON = 0;
    }

    /**
     * Constants for the right joystick button functions.
     */
    public static class RJoy
    {
        public static final int PREPARE_CLIMB_BUTTON = LJoy.PREPARE_CLIMB_BUTTON;
    }

    /**
     * Constants for the drive class.
     */
    public static class Drive
    {
        public static final int LEFT_MOTOR_ONE_PORT = 0;
        public static final int LEFT_MOTOR_TWO_PORT = 1;
        public static final int RIGHT_MOTOR_ONE_PORT = 2;
        public static final int RIGHT_MOTOR_TWO_PORT = 3;
   
        public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

        public static final int ULTRASONIC_PORT_1 = 5;
        public static final int ULTRASONIC_PORT_2 = 6;
        
        public static final int LEFT_ENCODER_PORT_ONE = 4;
        public static final int LEFT_ENCODER_PORT_TWO = 5;
        public static final int RIGHT_ENCODER_PORT_ONE = 2;
        public static final int RIGHT_ENCODER_PORT_TWO = 3;
        public static final double ENCODER_DISTANCE_PER_PULSE = Math.PI/90;
    
        public static final double TURN_P = 0.0;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.0;
        public static final double TURN_TOLERANCE = 2;

        public static final double DISTANCE_P = 0.0;
        public static final double DISTANCE_I = 0.0;
        public static final double DISTANCE_D = 0.0;
        public static final double DISTANCE_TOLERANCE = 1;
    }
    
    /**
     * Constants for the cargo transport class.
     */
    public static class CargoTransport
    {
        public static final int ROLLER_MOTOR_DEVICE_NUM = 6;
        public static final int ARM_MOTOR_DEVICE_NUM = 4;
        public static final int ARM_ENCODER_CHANNEL = 3;
        
        public static final double ARM_P = 0.0;
        public static final double ARM_I = 0.0;
        public static final double ARM_D = 0.0;
        public static final double ARM_PID_TOLERANCE = 3.0;
        public static final double ARM_MIN_INPUT = 0.0;
        public static final double ARM_MAX_INPUT = 200.0;

        public static final int ROCKET_BACK_ANGLE = 0;
        public static final int SHIP_BACK_ANGLE = 0;
        public static final int SHIP_FORWARD_ANGLE = 0;
        public static final int ARM_DOWN_ANGLE = 0;
    }

    /**
     * Constants for the panel transport class.
     */
    public static class PanelTransport
    {
        public static final int PUSHER_CHANNEL_1 = 5;
        public static final int PUSHER_CHANNEL_2 = 6;
        public static final int HOLDER_CHANNEL_1 = 7;
        public static final int HOLDER_CHANNEL_2 = 8;
    }

    /**
     * Constants for the pixy cam class.
     */
    public static class PixyCam
    {
        public static final SPI.Port PIXY_PORT = SPI.Port.kOnboardCS0; 
        public static final int PIXY_BUFFER_SIZE = 100;
        public static final int PIXY_CLOCKRATE = 1000000;
    }

    public static class Lidar
    {
        
        public static final SerialPort.Port LIDAR_PORT = SerialPort.Port.kUSB1;
        public static final int LIDAR_BAUD_RATE = 115200;
        public static final byte LIDAR_CMD_START = (byte) 0x01;
        public static final byte LIDAR_CMD_STOP = (byte) 0x02;
        public static final byte LIDAR_CMD_UPDATE = (byte) 0x03;

    }
}