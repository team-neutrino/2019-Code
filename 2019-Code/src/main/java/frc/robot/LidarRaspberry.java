/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.ByteBuffer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * Basic system for controlling the LiDAR subsystem
 * @author IndyPrieto
 */
public class LidarRaspberry {

    /**
     * Null Modem connection with RbPI
     */
    private SerialPort lidarConnection;

    /**
     * Navx access
     */

    private AHRS navX;

    /**
     * Message buffer
     */
    private byte[] message;


    //TODO: Add docs for constructor
    public LidarRaspberry(AHRS ahrs)
    {
        
        lidarConnection = new SerialPort(Constants.Lidar.LIDAR_BAUD_RATE,Constants.Lidar.LIDAR_PORT);
        lidarConnection.setWriteBufferSize(5); //Buffer is one control byte followed by the rotation of the robot
        navX = ahrs;

        message = new byte[5];

    }


    public void enable()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(navX.getYaw()+(float)180.0).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_START;
        lidarConnection.write(message, 5);
        lidarConnection.flush();
    }

    public void disable()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(navX.getYaw()+(float)180.0).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_STOP;
        lidarConnection.write(message, 5);
        lidarConnection.flush();
    }

    public void update()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(navX.getYaw()+(float)180.0).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_UPDATE;
        lidarConnection.write(message, 5);
        lidarConnection.flush();
    }


}