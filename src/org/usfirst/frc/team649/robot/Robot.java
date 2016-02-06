
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
		
	VideoCapture vcap;
	//private final static String[] GRIP_ARGS = new String[] {
//	        "/usr/local/frc/JRE/bin/java", "-jar",
//	        "/home/lvuser/grip.jar", "/home/lvuser/project.grip" };

	    //	private final NetworkTable grip = NetworkTable.getTable("grip");

	    @Override
	    public void robotInit() {
	    	System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
	    	try{
	    		vcap = new VideoCapture("http://root:admin@axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");
	    	}
	    	catch (Exception e){
	    		System.out.println("\n\n\nERROROROROROROROR WITH CAM");
	    		System.out.println(e.getMessage() + "\n\n\n");
	    	}
//	        if (!vcap.isOpened())  // if not success
//	        {
//	           System.out.println("Couldn't Open stream");
//	        } 
	        
	        /* Run GRIP in a new process */
//	        try {
//	            Runtime.getRuntime().exec(GRIP_ARGS);
//	        } catch (IOException e) {
//	            e.printStackTrace();
//	        }
	    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    @Override
	public void disabledInit(){

    }
	
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    @Override
	public void autonomousInit() {
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
//        /* Get published values from GRIP using NetworkTables */
//        for (double area : grip.getNumberArray("retroTapeReport/area", new double[0])) {
//            System.out.println("Got contour with area=" + area);
//        }
    }

    @Override
	public void teleopInit() {
    	System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
    	try{
    		vcap = new VideoCapture("http://root:admin@axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");
    	}
    	catch (Exception e){
    		System.out.println("\n\n\nERROROROROROROROR WITH CAM");
    		System.out.println(e.getMessage() + "\n\n\n");
    	}
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
	public void teleopPeriodic() {
    	
    	Mat image = new Mat();
    	Mat imageHSV = new Mat();
    	
    	vcap.read(image);
    	
    	Imgproc.cvtColor(image, imageHSV, Imgproc.COLOR_BGR2HSV);
    	
    	Core.inRange(imageHSV, new Scalar(110, 50, 50), new Scalar(130, 255, 255), imageHSV);
    	
    	Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(3, 3));
        Imgproc.erode(imageHSV, imageHSV, erode);
    	Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(3, 3));
        Imgproc.dilate(imageHSV, imageHSV, dilate);//dilate   
    	
    	List<MatOfPoint> contours = new ArrayList<>();
    	Mat hierarchy = new Mat();

    	// find contours
    	Imgproc.findContours(imageHSV, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

    	// if any contour exist...
    	if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
    	{
    		SmartDashboard.putNumber("Number of contours in image", contours.size());
	        // for each contour
	        for (int i = 0; i < contours.size(); i++)
	        {
	        	Imgproc.contourArea(contours.get(i));
	        }
    	}
    	//Imgproc.findContours(imageHSV, contours, hierarchy, mode, method);
    	
    	SmartDashboard.putNumber("Mat Height", image.height());
    	SmartDashboard.putNumber("Mat Width", image.width());
    	
    	
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    @Override
	public void testPeriodic() {
        LiveWindow.run();
    }
}
