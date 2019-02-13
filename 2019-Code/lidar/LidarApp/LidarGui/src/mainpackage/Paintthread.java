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
 * @author Team Neutrino
 * 
 *
 */

public class Paintthread extends Thread {
	
	final int UDP_PORT = 5800;
	
	
	private GraphicsContext Gc;
	private volatile boolean KillSwitch; //When set to true, causes the thread to end
	private DatagramPacket Packet;
	private byte[] MsgBuf;
	private String[] ParsedPacket; //Not really parsed...
	public DatagramSocket ServerSocket;
	
	private LinkedList<Double> Xlist;  //List of X points
	private LinkedList<Double> Ylist;  //List of Y points
	
	private LinkedList<Double> PXlist; //Buffers that contain last rotation, which prevents "flickering"
	private LinkedList<Double> PYlist;
	
	private double CurrentX;
	private double CurrentY;
	
	private int Aindex;
	
	public Paintthread(GraphicsContext gc) {
		
		//Do some messy initialization
		CurrentX = 0;
		CurrentY = 0;
		Gc = gc;
		Gc.setLineWidth(1);
		KillSwitch = false;
		System.out.println("Controller working...");
		ParsedPacket = new String[3];
		
		
		
		
		
		
		
	}
	
	public void kill() {
		KillSwitch = true;
	}
	
	public DatagramSocket startServer() {
		
		try {
			ServerSocket = new DatagramSocket(UDP_PORT); //Attempt to set up a socket server, on whatever port
			
			System.out.println("Socket Established");
			
			MsgBuf = new byte[21];
			
			Packet = new DatagramPacket(MsgBuf, MsgBuf.length);
			
			Xlist = new LinkedList<Double>();
			Ylist = new LinkedList<Double>();
			PXlist = new LinkedList<Double>();
			PYlist = new LinkedList<Double>();
		} catch (SocketException e) {

			e.printStackTrace();
			kill(); //Cause the thread to end if the socket creation failed
		}
		
		return ServerSocket;
	}
	
	private double clamp(double x, double l, double h) {
		return Math.max(Math.min(x, h), l);
	}
	
	public void run() {
		
		while(!Thread.currentThread().isInterrupted() && !KillSwitch) {
			
				
				
				
				try {
					ServerSocket.receive(Packet); //Wait for a sample from the RbPI
				} catch (Exception e) {
					e.printStackTrace();
				}
				
				ParsedPacket = (new String(MsgBuf)).split(","); //Turn the packet into something somewhat usable
				
				if(ParsedPacket[0].equals("1") && Aindex > 4) {
					
					
					
					Gc.clearRect(0, 0, Gc.getCanvas().getWidth(), Gc.getCanvas().getHeight()); //Clear screen
					
					
					//Draw some cool circles
					Gc.setStroke(Color.RED);
					Gc.strokeOval(250, 250, 100, 100);
					Gc.setStroke(Color.GRAY);
					Gc.strokeOval(200, 200, 200, 200);
					Gc.strokeOval(150, 150, 300, 300);
					
					//Draw last rotation
					if(PXlist.size()>1) {
						for(int i = 0; i < PXlist.size(); i++) {
							Gc.fillOval(PXlist.get(i), PYlist.get(i), 4, 4);
							
						}
					}
					
					Gc.setFill(Color.BLACK);
					
					
					Gc.beginPath(); //Might not need this
					
					//Draw current rotation
					Gc.moveTo(Xlist.get(Xlist.size()-1), Ylist.get(Ylist.size()-1));
					for(int i = 0; i < Xlist.size(); i++) {
						Gc.fillOval(Xlist.get(i), Ylist.get(i), 4, 4);
						
					}
					
					
					
					
					Gc.setStroke(Color.BLACK);
					Gc.stroke(); //See line 130
					
					PXlist.clear(); //Clear past rotation buffers
					PYlist.clear();
					
					PXlist.addAll(Xlist); //Copy current rotation buffers to past rotation buffers
					PYlist.addAll(Ylist);
					
					Xlist.clear(); //Clear current rotation buffers
					Ylist.clear();
					
				}
				//System.out.println(ParsedPacket[0] + " : " + Double.parseDouble(ParsedPacket[1]) + " : " + Double.parseDouble(ParsedPacket[2])); //Optional console display
				CurrentX = Double.parseDouble(ParsedPacket[1]); //Added in-between for ease to read.
				CurrentY = Double.parseDouble(ParsedPacket[2]);
				Xlist.add(clamp(CurrentX/10,-300,300)+300);
				Ylist.add(clamp(CurrentY/10,-300,300)+300);
				//Xlist.add(Math.min(Math.max(Double.parseDouble(ParsedPacket[1]), 0), 100.0));
				//Ylist.add(Math.min(Math.max(Double.parseDouble(ParsedPacket[2]), 0), 100.0));
				
				Aindex++;
				
				}
		
		if(Thread.currentThread().isInterrupted()) {
			Thread.currentThread().interrupt();
		}
		System.out.println("FATAL");
		ServerSocket.close();
	
	}
}
