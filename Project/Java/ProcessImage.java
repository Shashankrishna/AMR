import java.awt.Color;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import org.opencv.core.Core;

import data.Map;
import func.MapNotCompleteException;

public class ProcessImage {

	public static void main(String[] args) {
		// Load needed DLLs for OpenCV
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		// Create new Map (custom class) for image
		Map map = new Map();

		// Get image and load to map
		File f = new File("cameraCapture.png");
		map.input(f);
		
		
		// Process the image (if pass true, deskew image first)
		try {
			map.process(true);
		} catch (MapNotCompleteException e) { // Features not detected
			e.printStackTrace();
		}
		
		
		//map.setRobot(new Point(0, 0)); // Define Robot Starting Position if needed
		
		
		//--- Debug Outputs ---//
		Point[] points = map.getMapCorners();
		Point p;

		for (Point p1 : points) {
			// Print found corner locations
			System.out.println("Corner: " + p1.x + ", " + p1.y);
		}

		// Print width, height of detected area
		System.out.println("Width: " + map.getMapWidth());
		System.out.println("Height: " + map.getMapHeight());

		// Print robot starting location if needed
		//p = map.getRobot();
		//System.out.println("Robot: " + p.x + ", " + p.y);

		// Print found goal location
		p = map.getGoal();
		System.out.println("Goal: " + p.x + ", " + p.y);

		// Print number of obstacle points detected
		System.out.println("Obst Points: " + map.getObst().size());

		System.out.println("-------------------------------");
		//--- End Debug Outputs ---//


		// Remap image map to final workspace map
		Map workspace = Map.scaleTo(map, 400, 400);

		// Set starting location for robot (always at origin)
		workspace.setRobot(new Point(0, 0));


		//--- Debug Outputs ---//
		points = workspace.getMapCorners();

		for (Point p1 : points) {
			// Print found corner locations
			System.out.println("Corner: " + p1.x + ", " + p1.y);
		}

		// Print width, height of detected area
		System.out.println("Width: " + workspace.getMapWidth());
		System.out.println("Height: " + workspace.getMapHeight());

		// Print robot starting location
		p = workspace.getRobot();
		System.out.println("Robot: " + p.x + ", " + p.y);

		// Print found goal location
		p = workspace.getGoal();
		System.out.println("Goal: " + p.x + ", " + p.y);

		// Print number of obstacle points detected
		System.out.println("Obst Points: " + workspace.getObst().size());

		System.out.println("-------------------------------");
		//--- End Debug Outputs ---//
		
		
		// Save data to csv for use in other code
		// Save obstacle locations, goal location, and robot starting location
		try {
			FileWriter fw = new FileWriter(new File("data.csv"));
			fw.write("ObstX,ObstY,GoalX,GoalY,RobotX,RobotY\n");

			if (workspace.getObst().size() <= 0) {
				fw.write("," + ",");

				Point p1 = workspace.getGoal();
				fw.write(p1.x + "," + p1.y + ",");

				p1 = workspace.getRobot();
				fw.write(p1.x + "," + p1.y + "\n");
			}

			for (int i = 0; i < workspace.getObst().size(); i++) {
				Point p1 = workspace.getObst().get(i);

				if (i == 0) {
					fw.write(p1.x + "," + p1.y + ",");

					p1 = workspace.getGoal();
					fw.write(p1.x + "," + p1.y + ",");

					p1 = workspace.getRobot();
					fw.write(p1.x + "," + p1.y + "\n");
				}
				else {
					fw.write(p1.x + "," + p1.y + ",,,,\n");
				}
			}

			fw.flush();
			fw.close();

		} catch (IOException e) {
			e.printStackTrace();
		}

		
		
		// Create a new image representing the workspace, useful for debug
		BufferedImage workspaceImg = new BufferedImage(400, 400, BufferedImage.TYPE_3BYTE_BGR);

		for (int i = 0; i < workspaceImg.getWidth(); i++) {
			for (int j = 0; j < workspaceImg.getHeight(); j++) {
				workspaceImg.setRGB(i, j, Color.white.getRGB());
			}
		}

		p = workspace.getRobot();
		workspaceImg.setRGB(p.x, p.y, Color.red.getRGB());

		p = workspace.getGoal();
		workspaceImg.setRGB(p.x, p.y, Color.green.getRGB());

		ArrayList<Point> obsts = workspace.getObst();

		for (Point p1 : obsts) {
			if ( (p1.x >= 0) && (p1.x < workspaceImg.getWidth()) 
					&& (p1.y >= 0) && (p1.y < workspaceImg.getHeight()) ) { // Ignore out of bounds data
				workspaceImg.setRGB(p1.x, p1.y, Color.black.getRGB());
			}
		}

		try {
			ImageIO.write(workspaceImg, "png", new File("workspaceImage.png"));
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

}
