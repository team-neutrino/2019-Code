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
    	FXMLLoader loader = new FXMLLoader(getClass().getResource("LidarWindow.fxml"));
        Parent root = loader.load();
        CanvasController Ctrlr = (CanvasController) loader.getController();
        primaryStage.setTitle("Hello World");
        primaryStage.setScene(new Scene(root, 700, 700));
        primaryStage.show();
        
        primaryStage.setOnCloseRequest(event -> {Ctrlr.closing();});
    }


    public static void main(String[] args) {
        launch(args);
    }
}