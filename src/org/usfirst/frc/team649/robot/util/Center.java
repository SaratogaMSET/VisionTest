package org.usfirst.frc.team649.robot.util;

public class Center {
	public double x, y;
	public Center(double _x, double _y){
		x = _x;
		y = _y;
	}
	
	public boolean equals(Center a){
		return a.x == this.x && a.y == this.y;
	}
	
	
}
