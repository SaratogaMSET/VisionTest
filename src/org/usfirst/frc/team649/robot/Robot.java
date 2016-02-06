
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static double WIDTH_TARGET = 18.5; //in
	public static double STANDARD_VIEW_ANGLE = 0.454885;//0.9424778; //radians, for an Axis Camera 206 /////...54 degrees
		
	VideoCapture vcap;
	Mat image, imageHSV, erode, dilate, hierarchy;
	List<MatOfPoint> contours;
	
	
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
    	Scheduler.getInstance().run();
    	
    	
    	image = new Mat();
    	imageHSV = new Mat();
    	
    	vcap.read(image);
    	
    	Imgproc.cvtColor(image, imageHSV, Imgproc.COLOR_BGR2HSV);
    	
    	Core.inRange(imageHSV, new Scalar(78, 124, 213), new Scalar(104, 255, 255), imageHSV);
    	
//    	erode = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(3, 3));
//        Imgproc.erode(imageHSV, imageHSV, erode);
//    	dilate = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(3, 3));
//        Imgproc.dilate(imageHSV, imageHSV, dilate);//dilate   
    	
    	Imgproc.GaussianBlur(imageHSV, imageHSV, new Size(3,3), 0);
    	
    	contours = new ArrayList<>();
    	hierarchy = new Mat();

    	// find contours
    	Imgproc.findContours(imageHSV, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

    	// if any contour exist...
    	if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
    	{
    		int largest = 0;
    		
	        // for each contour, find the biggest
	        for (int i = 0; i < contours.size(); i++)
	        {
	        	double area = Imgproc.contourArea(contours.get(i));
	        	//greater than min size and greater than the last biggest
	        	if (area > 100.0    &&    area > Imgproc.contourArea(contours.get(largest))){
	        		largest = i;
		        	//NetworkTable tab = NetworkTable.getTable("Obj " + i);
	        		
	        		//Center:    mu.m10()/mu.m00() , mu.m01()/mu.m00()
//		        	tab.putString(name + " Center: ", "("+ mu.get_m10()/mu.get_m00() + ", " + mu.get_m01()/mu.get_m00() + ")");
//		        	tab.putNumber(name + " Area: ", Imgproc.contourArea(contours.get(i)));
//		        	tab.putNumber("Obj width: ", contours.get(i).width());
//		        	tab.putNumber("Obj height: ", contours.get(i).height());
	        		
		        	//SendableTable contourObj = new SendableTable("Obj " + i, tab);
		        	//SmartDashboard.putData("Obj " + i, contourObj);
	        	}
	        }
	        
	        //print details of the biggest one
        	//String name = "Obj " + largest;
        	Rect r = Imgproc.boundingRect(contours.get(largest));
        	Moments mu = Imgproc.moments(contours.get(largest));
        	
        	//"("+ mu.get_m10()/mu.get_m00() + ", " + mu.get_m01()/mu.get_m00() + ")");
        	
        	//ASSUME LARGEST is the target, now calc dist
        	
        	double dist = calcDistAxis206(r.width, WIDTH_TARGET, 320, STANDARD_VIEW_ANGLE);

        	SmartDashboard.putNumber("Obj 0 Center X: ", mu.get_m10()/mu.get_m00());
        	SmartDashboard.putNumber("Obj 0 Center Y: ", mu.get_m01()/mu.get_m00());
        	SmartDashboard.putNumber("Obj 0 Area: ", Imgproc.contourArea(contours.get(largest)));
        	SmartDashboard.putNumber("Obj 0 width: ", r.width);
        	SmartDashboard.putNumber("Obj 0 height: ", r.height);
        	SmartDashboard.putNumber("Obj 0 Distance: ", dist);
	        
    	}
    	else{
    		SmartDashboard.putNumber("Obj 0 Center X: ", 0);
        	SmartDashboard.putNumber("Obj 0 Center Y: ", 0);
        	SmartDashboard.putNumber("Obj 0 Area: ", 0);
        	SmartDashboard.putNumber("Obj 0 width: ", 0);
        	SmartDashboard.putNumber("Obj 0 height: ", 0);
        	SmartDashboard.putNumber("Obj 0 Distance: ", 0);
    	}
    	
    	
    	SmartDashboard.putNumber("Number of contours in image", contours.size());
    	SmartDashboard.putNumber("Mat Height", image.height());
    	SmartDashboard.putNumber("Mat Width", image.width());
    	
        //mem save
        image.release();;
    	imageHSV.release();
//    	erode.release();
//    	dilate.release();
    	hierarchy.release();
        System.gc();
    }

    public double calcDistAxis206(double obj_pix, double obj_in, double view_pix, double max_cam_angle){
    	return view_pix * obj_in / (2*Math.tan(max_cam_angle) * obj_pix);
    }
    
    /**
     * This function is called periodically during test mode
     */
    @Override
	public void testPeriodic() {
        LiveWindow.run();
    }
    
    
    ///***********************************************//
    public class SendableTable implements Sendable{
    	ITable table;
    	String t;
		SendableTable(String type,ITable subtable){
			t = type;
			table = subtable;
		}

		@Override
		public void initTable(ITable subtable) {
			// TODO Auto-generated method stub
			table = subtable;
			
		}

		@Override
		public ITable getTable() {
			// TODO Auto-generated method stub
			return table;
		}

		@Override
		public String getSmartDashboardType() {
			// TODO Auto-generated method stub
			return t;
		}
    }
}
