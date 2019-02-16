/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.SerialPort;

import java.net.*;
/**
 * Basic system for controlling the LiDAR subsystem
 * @author IndyPrieto
 */
public class LidarRaspberry {

    /**
     * Drive access
     */
    private Drive drv;

    /**
     * Message buffer
     */
    private byte[] message;


    /**
     * Datagram (UDP) Socket
     */
    private DatagramSocket clientSocket;

    /**
     * Datagram packet
     */
    private DatagramPacket packet;

    /**
     * IPv4 of the RbPI
     */
    private InetAddress piAddr;


    /**
     * Creates a link to the Raspberry Pi to control the lidar
     * @param drive
     *  The drive instance
     */
    public LidarRaspberry(Drive drive)
    {

        drv = drive;

        message = new byte[5];

        

        try
        {
            piAddr = InetAddress.getByName(Constants.Lidar.LIDAR_ADDR);
            clientSocket = new DatagramSocket(Constants.Lidar.LIDAR_PORT);
            packet = new DatagramPacket(message, 5, piAddr, Constants.Lidar.LIDAR_PORT);
        }
        catch (Exception e)
        {
            e.getStackTrace();
        }

    }

    /**
     * Gets the current robot angle
     * @return
     *  Current robot angle
     */
    private float getAngle()
    {
        return (float) (drv.getNavxAngle() + 180);

    }

    /**
     * Tells the Lidar to start spinning
     */
    public void enable()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(getAngle()).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_START;
        try 
        {
            clientSocket.send(packet);
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
        }
            
    }

    /**
     * Tells the Lidar to stop spinning
     */
    public void disable()
    {   
        System.arraycopy(ByteBuffer.allocate(4).putFloat(getAngle()).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_STOP;
        try 
        {
            clientSocket.send(packet);
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
        }   
    }

    /**
     * Sends the current angle to the Raspberry PI
     */
    public void update()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(getAngle()).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_UPDATE;
        try 
        {
            clientSocket.send(packet);
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
        }
    }


}