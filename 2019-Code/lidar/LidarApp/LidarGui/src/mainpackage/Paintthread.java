/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package mainpackage;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.LinkedList;
import java.util.Random;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;

/**
 * 
 * @author IndyPrieto
 * 
 *
 */

public class PaintThread extends Thread 
{
	
	final int UDP_PORT = 5800;
	
	
	private GraphicsContext gc;
	private volatile boolean killSwitch; //When set to true, causes the thread to end
	private DatagramPacket packet;
	private byte[] msgBuf;
	private String[] parsedPacket; //Not really parsed...
	public DatagramSocket serverSocket;
	
	private LinkedList<Double> xList;  //List of X points
	private LinkedList<Double> yList;  //List of Y points
	
	private LinkedList<Double> pxList; //Buffers that contain last rotation, which prevents "flickering"
	private LinkedList<Double> pyList;
	
	private double currentX;
	private double currentY;
	
	private int sIndex;
	
    public PaintThread(GraphicsContext gc) 
    {
		
		//Do some messy initialization
		currentX = 0;
		currentY = 0;
		gc = gc;
		gc.setLineWidth(1);
		killSwitch = false;
		System.out.println("Controller working...");
		parsedPacket = new String[3];
	}
	
	public void kill() {
		killSwitch = true;
	}
	
	public DatagramSocket startServer() {
		
        try 
        {
			serverSocket = new DatagramSocket(UDP_PORT); //Attempt to set up a socket server, on whatever port
			
			System.out.println("Socket Established");
			
			msgBuf = new byte[21];
			
			packet = new DatagramPacket(MsgBuf, MsgBuf.length);
			
			xList = new LinkedList<Double>();
			yList = new LinkedList<Double>();
			pxList = new LinkedList<Double>();
			pyList = new LinkedList<Double>();
        } 
        catch (SocketException e) 
        {

			e.printStackTrace();
			kill(); //Cause the thread to end if the socket creation failed
		}
		
		return serverSocket;
	}
	
    private double clamp(double x, double l, double h)
    {
		return Math.max(Math.min(x, h), l);
	}
	
    public void run()
    {
		
        while(!Thread.currentThread().isInterrupted() && !killSwitch)
        {
			
				
				
				
                try 
                {
					serverSocket.receive(packet); //Wait for a sample from the RbPI
                } 
                catch (Exception e) 
                {
					e.printStackTrace();
				}
				
				parsedPacket = (new String(msgBuf)).split(","); //Turn the packet into something somewhat usable
				
                if(parsedPacket[0].equals("1") && sIndex > 4) 
                {
					
					
					
					gc.clearRect(0, 0, gc.getCanvas().getWidth(), gc.getCanvas().getHeight()); //Clear screen
					
					
					//Draw some cool circles
					gc.setStroke(Color.RED);
					gc.strokeOval(250, 250, 100, 100);
					gc.setStroke(Color.GRAY);
					gc.strokeOval(200, 200, 200, 200);
					gc.strokeOval(150, 150, 300, 300);
					
					//Draw last rotation
                    if(pxList.size()>1) 
                    {
                        for(int i = 0; i < pxList.size(); i++) 
                        {
							gc.fillOval(pxList.get(i), pyList.get(i), 4, 4);
							
						}
					}
					
					gc.setFill(Color.BLACK);
					
					
					gc.beginPath(); //Might not need this
					
					//Draw current rotation
					gc.moveTo(xList.get(xList.size()-1), yList.get(yList.size()-1));
                    for(int i = 0; i < xList.size(); i++) 
                    {
						gc.fillOval(xList.get(i), yList.get(i), 4, 4);
						
					}
					
					
					
					
					gc.setStroke(Color.BLACK);
					gc.stroke(); //See line 130
					
					pxList.clear(); //Clear past rotation buffers
					pyList.clear();
					
					pxList.addAll(xList); //Copy current rotation buffers to past rotation buffers
					pyList.addAll(yList);
					
					xList.clear(); //Clear current rotation buffers
					yList.clear();
					
				}
				//System.out.println(ParsedPacket[0] + " : " + Double.parseDouble(ParsedPacket[1]) + " : " + Double.parseDouble(ParsedPacket[2])); //Optional console display
				currentX = Double.parseDouble(ParsedPacket[1]); //Added in-between for ease to read.
				currentY = Double.parseDouble(ParsedPacket[2]);
				xList.add(clamp(currentX/10,-300,300)+300);
				yList.add(clamp(currentY/10,-300,300)+300);
				//Xlist.add(Math.min(Math.max(Double.parseDouble(ParsedPacket[1]), 0), 100.0));
				//Ylist.add(Math.min(Math.max(Double.parseDouble(ParsedPacket[2]), 0), 100.0));
				
				sIndex++;
				
				}
		
        if(Thread.currentThread().isInterrupted()) 
        {
			Thread.currentThread().interrupt();
		}
		System.out.println("FATAL");
		serverSocket.close();
	
	}
}
