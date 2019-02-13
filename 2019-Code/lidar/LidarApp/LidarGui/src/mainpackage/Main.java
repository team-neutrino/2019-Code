package mainpackage;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

/**
 * 
 * @author Team Neutrino
 * 
 *
 */

public class Main extends Application {

    @Override
    public void start(Stage primaryStage) throws Exception{
    	
    	FXMLLoader loader = new FXMLLoader(getClass().getResource("LidarWindow.fxml")); //Loads GUI XML file (FXML)
        
    	Parent root = loader.load();
    	
        CanvasController Ctrlr = (CanvasController) loader.getController(); //Gets access to the script for the GUI
        primaryStage.setTitle("Team neutrino LIDAR app");
        primaryStage.setScene(new Scene(root, 700, 700)); 
        primaryStage.show(); //Show app window
        
        primaryStage.setOnCloseRequest(event -> {Ctrlr.closing();}); //Activates the closing function in the script when the window is closed.
    }


    public static void main(String[] args) {
        launch(args);
    }
}