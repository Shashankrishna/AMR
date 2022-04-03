package func;

import java.awt.Color;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Analyze {

	// Find corners in image
	static public Point[] findCorners(BufferedImage img) throws MapNotCompleteException {

		// Create new Mat to hold corner data
		Mat m = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);

		byte[] white = {(byte) 0xFF}; // White pixel
		byte[] black = {(byte) 0x00}; // Black pixel

		// Scan over image
		for (int i = 0; i < img.getWidth(); i++) {
			for (int j = 0; j < img.getHeight(); j++) {

				// Check color of pixel
				int pixel = img.getRGB(i, j);
				Color c = new Color(pixel);

				// Use hue, saturation, brightness
				float[] hsb = Color.RGBtoHSB(c.getRed(), c.getGreen(), c.getBlue(), null);

				float hue = hsb[0]*360.0f;
				float sat = hsb[1];
				float val = hsb[2];

				// Check for hue, sat, val, in right ranges
				if ((sat >= 0.8f) && (val >= 0.5f)) {
					if (Math.abs(hue-22.0f) < 10.0f) {
						m.put(j, i, white); // Put white pixel in Mat
					}
					else {
						m.put(j, i, black); // Put black pixel in Mat
					}
				}

			}

		}

		// OpenCV find contours
		List<MatOfPoint> conts = new ArrayList<MatOfPoint>();
		Imgproc.findContours(m, conts, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		ArrayList<Point> foundPoints = new ArrayList<Point>();

		// Get centers of contours
		for (MatOfPoint c : conts) {
			Moments p = Imgproc.moments(c);

			int x = (int) (p.get_m10() / p.get_m00());
			int y = (int) (p.get_m01() / p.get_m00());

			if (x != 0 && y != 0) {
				foundPoints.add(new Point(x,y));
			}

		}

		// Check that corners were found
		if (foundPoints.size() < 4) {
			throw new MapNotCompleteException("Missing corners, current: " + foundPoints.size());
		}
		else if (foundPoints.size() > 4) {
			throw new MapNotCompleteException("Too many corners, current: " + foundPoints.size());
		}

		Point[] corners = new Point[4];

		// Order corners correctly
		for (Point p : foundPoints) {
			int h2 = img.getHeight()/2;
			int w2 = img.getWidth()/2;

			if (p.x < w2 && p.y < h2) { // Top Left (0)
				corners[0] = p;
			}
			else if (p.x > w2 && p.y < h2) { // Top Right (1)
				corners[1] = p;
			}
			else if (p.x < w2 && p.y > h2) { // Bottom Left (2)
				corners[2] = p;
			}
			else if (p.x > w2 && p.y > h2) { // Bottom Right (3)
				corners[3] = p;
			}
		}

		return corners;
	}

	// Find goal in image
	static public Point findGoal(BufferedImage img) throws MapNotCompleteException {

		// Create new Mat to hold corner data
		Mat m = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);

		byte[] white = {(byte) 0xFF}; // White pixel
		byte[] black = {(byte) 0x00}; // Black pixel

		// Scan over image
		for (int i = 0; i < img.getWidth(); i++) {
			for (int j = 0; j < img.getHeight(); j++) {

				// Check color of pixel
				int pixel = img.getRGB(i, j);
				Color c = new Color(pixel);

				// Use hue, saturation, brightness
				float[] hsb = Color.RGBtoHSB(c.getRed(), c.getGreen(), c.getBlue(), null);

				float hue = hsb[0]*360.0f;
				float sat = hsb[1];
				float val = hsb[2];

				// Check for hue, sat, val, in right ranges
				if ((sat >= 0.8f) && (val >= 0.5f)) {
					if (Math.abs(hue-125.0f) < 10.0f) {
						m.put(j, i, white); // Put white pixel in Mat
					}
					else {
						m.put(j, i, black); // Put black pixel in Mat
					}
				}

			}

		}

		// OpenCV find contours
		List<MatOfPoint> conts = new ArrayList<MatOfPoint>();
		Imgproc.findContours(m, conts, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		ArrayList<Point> foundPoints = new ArrayList<Point>();

		// Get centers of contours
		for (MatOfPoint c : conts) {
			Moments p = Imgproc.moments(c);

			int x = (int) (p.get_m10() / p.get_m00());
			int y = (int) (p.get_m01() / p.get_m00());

			if (x != 0 && y != 0) {
				foundPoints.add(new Point(x,y));
			}

		}

		// Check that goal was found
		if (foundPoints.size() < 1) {
			throw new MapNotCompleteException("Missing goal, current: " + foundPoints.size());
		}
		else if (foundPoints.size() > 1) {
			throw new MapNotCompleteException("Too many goals, current: " + foundPoints.size());
		}

		return foundPoints.get(0);
	}

	
	// Get width and height from corner data
	// Return the largest width and height only
	public static int[] findWH(Point[] corners) {
		int[] wh = new int[2];

		double width1 = corners[0].distance(corners[1]);
		double width2 = corners[2].distance(corners[3]);
		wh[0] = (int) Math.max(width1, width2);

		double height1 = corners[0].distance(corners[2]);
		double height2 = corners[1].distance(corners[3]);
		wh[1] = (int) Math.max(height1, height2);

		return wh;
	}

	// Correct skew/perspective issues with image
	public static Mat correctPerspective(Mat m, int[] wh, Point[] corners) {

		// Use found corners as a source points
		List<org.opencv.core.Point> tmp = new ArrayList<org.opencv.core.Point>();

		for (Point p : corners) {
			tmp.add(new org.opencv.core.Point(p.x, p.y));
		}

		MatOfPoint2f src = new MatOfPoint2f();
		src.fromList(tmp);

		tmp.clear();

		// Use starting corner and width and height as destination points
		Point O = corners[0];

		tmp.add(new org.opencv.core.Point(O.x, O.y));
		tmp.add(new org.opencv.core.Point(O.x+wh[0], O.y));
		tmp.add(new org.opencv.core.Point(O.x, O.y+wh[1]));
		tmp.add(new org.opencv.core.Point(O.x+wh[0], O.y+wh[1]));

		MatOfPoint2f dst = new MatOfPoint2f();
		dst.fromList(tmp);

		// Calculate adjustment matrix
		Mat homog = Calib3d.findHomography(src, dst, Calib3d.RANSAC, 3.0);

		Mat out = new Mat(m.size(), m.type()); // New Mat

		// Apply adjustment matrix over image
		Imgproc.warpPerspective(m, out, homog, m.size());

		return out;
	}

	// Find obstacles
	public static ArrayList<Point> findObstacles(Mat m) {
		ArrayList<Point> list = new ArrayList<Point>();

		// Get grayscale image
		Imgproc.cvtColor(m, m, Imgproc.COLOR_BGR2GRAY);

		// Scan over image
		for (int i = 0; i < m.width(); i++) {
			for (int j = 0; j < m.height(); j++) {
				double[] d = m.get(j, i);

				if (d[0] < 10) { // Get points that are in range
					list.add(new Point(i,j));
				}
			}
		}

		return list;
	}

	// Convert from BufferedImage to Mat
	public static Mat BF2Mat(BufferedImage img) {
		byte[] pixels = ((DataBufferByte) img.getRaster().getDataBuffer()).getData();
		Mat m = new Mat(img.getHeight(), img.getWidth(), CvType.CV_8UC3);
		m.put(0, 0, pixels);
		return m;
	}

	// Convert from Mat to BufferedImage
	public static BufferedImage Mat2BF(Mat m) throws Exception {        
		MatOfByte mob = new MatOfByte();
		Imgcodecs.imencode(".jpg", m, mob);
		byte ba[] = mob.toArray();
		BufferedImage img = ImageIO.read(new ByteArrayInputStream(ba));
		return img;
	}

}
