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
	private GraphicsContext Gc;
	private volatile boolean KillSwitch;
	private Random Rng;
	private DatagramPacket Packet;
	private byte[] MsgBuf;
	private String[] ParsedPacket;
	public DatagramSocket ServerSocket;
	private volatile boolean Got_Sync;
	private LinkedList<Double> Xlist;
	private LinkedList<Double> Ylist;
	private LinkedList<Double> PXlist;
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
		System.out.println("inits");
		ParsedPacket = new String[3];
		Rng = new Random();
		
		//Attempt to set up a socket server
		
		
		
		
		
	}
	
	public void kill() {
		KillSwitch = true;
	}
	
	public DatagramSocket startServer() {
		try {
			ServerSocket = new DatagramSocket(5800);
			
			System.out.println("Socket Established");
			
			MsgBuf = new byte[21];
			
			Packet = new DatagramPacket(MsgBuf, MsgBuf.length);
			
			Xlist = new LinkedList<Double>();
			Ylist = new LinkedList<Double>();
			PXlist = new LinkedList<Double>();
			PYlist = new LinkedList<Double>();
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			kill();
		}
		
		return ServerSocket;
	}
	
	private double clamp(double x, double l, double h) {
		return Math.max(Math.min(x, h), l);
	}
	
	public void run() {
		
		while(!Thread.currentThread().isInterrupted() && !KillSwitch) {
			
				
				
				
				try {
					ServerSocket.receive(Packet);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				ParsedPacket = (new String(MsgBuf)).split(",");
				
				if(ParsedPacket[0].equals("1") && Aindex > 4) {
					
					
					
					Gc.clearRect(0, 0, Gc.getCanvas().getWidth(), Gc.getCanvas().getHeight());
					
					
					Gc.setStroke(Color.RED);
					Gc.strokeOval(250, 250, 100, 100);
					Gc.setStroke(Color.GRAY);
					
					Gc.strokeOval(200, 200, 200, 200);
					Gc.strokeOval(150, 150, 300, 300);
					
					
					if(PXlist.size()>1) {
						for(int i = 0; i < PXlist.size(); i++) {
							Gc.fillOval(PXlist.get(i), PYlist.get(i), 4, 4);
							
						}
					}
					
					Gc.setFill(Color.BLACK);
					
					//System.out.println("Following packet is sync packet!");
					Gc.beginPath();
					Gc.moveTo(Xlist.get(Xlist.size()-1), Ylist.get(Ylist.size()-1));
					for(int i = 0; i < Xlist.size(); i++) {
						//Gc.lineTo(Xlist.get(i),Ylist.get(i));
						Gc.fillOval(Xlist.get(i), Ylist.get(i), 4, 4);
						
					}
					
					
					
					
					Gc.setStroke(Color.BLACK);
					Gc.stroke();
					PXlist.clear();
					PYlist.clear();
					PXlist.addAll(Xlist);
					PYlist.addAll(Ylist);
					Xlist.clear();
					Ylist.clear();
					
				}
				//System.out.println(ParsedPacket[0] + " : " + Double.parseDouble(ParsedPacket[1]) + " : " + Double.parseDouble(ParsedPacket[2]));
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
