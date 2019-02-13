package mainpackage;

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
 * @author Team Neutrino
 * 
 *
 */

public class CanvasController {

    @FXML
    private Canvas LidarCanvas;
    
    private GraphicsContext gc;
    
    private Thread mt;
    private Paintthread pt;
    
    private DatagramSocket hook;
    
    private boolean ready;
    
    @FXML
    private void drawCanvas(ActionEvent event) {
        
    }

    public void initialize() {
    	
        gc = LidarCanvas.getGraphicsContext2D();
        
        pt = new Paintthread(gc);
        mt = new Thread(pt);
        
        hook = pt.startServer();
        mt.start();
        
    }
    
    public void closing() {
    	System.out.println("TERMINATING");
    	try {
			hook.close();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			
			e.printStackTrace();
		}

    	pt.kill();
    }
    
}