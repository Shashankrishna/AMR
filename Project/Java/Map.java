package data;

import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

import javax.imageio.ImageIO;

import org.opencv.core.Mat;

import func.Analyze;
import func.MapNotCompleteException;

public class Map {

	// Custom Class
	// Holds data from images after processing to create workspace maps
	// Also supports translating between workspace sizes

	// Variables
	private BufferedImage raw; // Base imae
	private Mat img; // Image in OpenCV Mat
	private Mat skewCorrectImg; // Image corrected for skew (if used)

	private Point[] mapCorners; // Corners of workspace
	private int mapWidth; // Width of workspace
	private int mapHeight; // Height of workspace

	private Point robot; // Robot location
	private float robotTheta; // Robot orientation (not used)

	private Point goal; // Goal location

	private ArrayList<Point> obst; // Obst. location

	public Map() { // Constructor
		this.raw = null;

		this.mapCorners = new Point[4];

		this.robot = new Point();
		this.robotTheta = 0;

		this.goal = new Point();

		this.obst = new ArrayList<Point>();
	}

	public void input(BufferedImage img) { // Input passed image
		this.raw = img;
	}

	public void input(File f) { // Input, read image from file
		try {
			this.raw = ImageIO.read(f);
		} catch (IOException e) {
			this.raw = null;
			e.printStackTrace();
		}
	}

	// Process image to determine features (with or without correcting for skew)
	// Throws MapNotCompleteException if features are missing
	public void process(boolean doCorrection) throws MapNotCompleteException {
		if (this.raw == null) throw new MapNotCompleteException("Image Empty");

		// Convert to OpenCV Mat
		this.img = Analyze.BF2Mat(raw); 

		// Find initial corners
		this.mapCorners = Analyze.findCorners(this.raw);

		// Calculate/store width and height
		int[] wh = Analyze.findWH(this.mapCorners);
		this.mapWidth = wh[0]; this.mapHeight = wh[1];

		if (doCorrection) { // Correct for skew

			// Correct image perspective
			this.skewCorrectImg = Analyze.correctPerspective(this.img, wh, this.mapCorners);

			// Find goal location
			try {
				this.goal = Analyze.findGoal(Analyze.Mat2BF(this.skewCorrectImg));
			} catch (Exception e) { e.printStackTrace(); }

			// Find obstacle locations
			this.obst = Analyze.findObstacles(this.skewCorrectImg);
		}

		else { // Do not correct for skew
			// Find goal location
			this.goal = Analyze.findGoal(this.raw);
			// Find obstacle locations
			this.obst = Analyze.findObstacles(this.img);
		}

	}

	// Getters, Setters

	public Point[] getMapCorners() {
		return mapCorners;
	}

	public void setMapCorners(Point[] mapCorners) {
		this.mapCorners = mapCorners;
	}

	public int getMapWidth() {
		return mapWidth;
	}

	public void setMapWidth(int mapWidth) {
		this.mapWidth = mapWidth;
	}

	public int getMapHeight() {
		return mapHeight;
	}

	public void setMapHeight(int mapHeight) {
		this.mapHeight = mapHeight;
	}

	public Point getRobot() {
		return robot;
	}

	public void setRobot(Point robot) {
		this.robot = robot;
	}

	public float getRobotTheta() {
		return robotTheta;
	}

	public void setRobotTheta(float robotTheta) {
		this.robotTheta = robotTheta;
	}

	public Point getGoal() {
		return goal;
	}

	public void setGoal(Point goal) {
		this.goal = goal;
	}

	public ArrayList<Point> getObst() {
		return obst;
	}

	public void setObst(ArrayList<Point> obst) {
		this.obst = obst;
	}

	public BufferedImage getRaw() {
		return this.raw;
	}

	public BufferedImage getSkewCorrectImg() throws Exception{
		return Analyze.Mat2BF(skewCorrectImg);
	}

	// Statics

	// Scale a workspace to a new width and height
	// Transforms all features inside as well
	public static Map scaleTo(Map m, int width, int height) {
		Map map = new Map();

		// Creat/set new corner locations
		Point[] corners = {new Point(0, 0), new Point(width, 0), new Point(0, height), new Point(width, height)};
		map.setMapCorners(corners);

		// Get opposite corners in existing map
		Point c1 = m.getMapCorners()[0];
		Point c2 = new Point(c1.x + m.getMapWidth(), c1.y + m.getMapHeight());

		Point p;
		int i, j;

		// Remap robot location
		p = m.getRobot();
		i = p.x; j = p.y;

		i = remap(i, c1.x, c2.x, 0, width);
		j = remap(j, c1.y, c2.y, 0, height);

		map.setRobot(new Point(i, j));

		// Remap goal location
		p = m.getGoal();
		i = p.x; j = p.y;

		i = remap(i, c1.x, c2.x, 0, width);
		j = remap(j, c1.y, c2.y, 0, height);

		map.setGoal(new Point(i, j));

		// Remap obstacle location
		HashSet<Point> set = new HashSet<Point>();

		for (Point p1 : m.getObst()) {
			i = p1.x; j = p1.y;

			i = remap(i, c1.x, c2.x, 0, width);
			j = remap(j, c1.y, c2.y, 0, height);

			set.add(new Point(i, j));
		}

		ArrayList<Point> obstP = new ArrayList<Point>();
		obstP.addAll(set); // Save only unique points
		map.setObst(obstP);

		return map;
	}

	// Privates

	// Linear transform
	private static int remap(int x, int in_min, int in_max, int out_min, int out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

}
