/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.ByteBuffer;
import java.io.IOException;
import java.net.*;

/**
 * Basic system for controlling the LiDAR subsystem
 * 
 * @author IndyPrieto
 * 
 */
public class LidarRaspberry 
{
    /**
     * Drive access
     */
    private Drive drive;

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
        this.drive = drive;

        message = new byte[5];

        try
        {
            piAddr = InetAddress.getByName(Constants.Lidar.LIDAR_ADDR);
            clientSocket = new DatagramSocket(Constants.Lidar.LIDAR_PORT);
            packet = new DatagramPacket(message, 5, piAddr, Constants.Lidar.LIDAR_PORT);
        }
        catch (IOException e)
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
        return (float) (drive.getNavxAngle()%360);
    }

    /**
     * Tells the Lidar to start spinning
     */
    public void enable()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(getAngle()).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_START;
        //message[0] = (byte) 0x00;
        //message[1] = (byte) 0x00;
        //message[2] = (byte) 0x00;
        //message[3] = (byte) 0x00;
        //message[4] = (byte) 0x01;
        try 
        {
            clientSocket.send(packet);
        } 
        catch (IOException e) 
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

        //message[0] = (byte) 0x00;
        //message[1] = (byte) 0x00;
        //message[2] = (byte) 0x00;
        //message[3] = (byte) 0x00;
        //message[4] = (byte) 0x02;
        try 
        {
            clientSocket.send(packet);
        } 
        catch (IOException e) 
        {
            e.printStackTrace();
        }   
    }

    /**
     * Sends the current angle of the robot relative to the 
     * field to the Raspberry PI
     */
    public void update()
    {
        System.arraycopy(ByteBuffer.allocate(4).putFloat(getAngle()).array(), 0, message, 0, 4);
        message[4] = Constants.Lidar.LIDAR_CMD_UPDATE;
        
        try 
        {
            clientSocket.send(packet);
        } 
        catch(IOException e)
        {
            e.printStackTrace();
        }
    }
}