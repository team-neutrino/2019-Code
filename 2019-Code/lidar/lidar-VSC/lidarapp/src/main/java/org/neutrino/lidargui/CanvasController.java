/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package org.neutrino.lidargui;

import java.net.DatagramSocket;


import java.net.SocketException;
import java.util.ResourceBundle;

import javax.print.DocFlavor.URL;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import javafx.scene.canvas.Canvas;

/**
 * 
 * @author IndyPrieto
 * 
 *
 */

public class CanvasController 
{

    @FXML
    private Canvas lidarCanvas;
    
    private GraphicsContext gc;
    
    private Thread mt;
    private PaintThread pt;
    
    private DatagramSocket hook;
    
    private boolean ready;
    
    @FXML
    private void drawCanvas(ActionEvent event)
    {
        
    }

    public void initialize() 
    {
    	
        gc = lidarCanvas.getGraphicsContext2D();
        
        pt = new PaintThread(gc);
        mt = new Thread(pt);
        
        hook = pt.startServer();
        mt.start();
        
    }
    
    //Causes the paintthread to terminate
    public void closing() 
    {
    	System.out.println("TERMINATING");
        try 
        {
			hook.close();
        } 
        catch (Exception e) 
        {
			// TODO Auto-generated catch block
			
			e.printStackTrace();
		}

    	pt.kill();
    }
    
}