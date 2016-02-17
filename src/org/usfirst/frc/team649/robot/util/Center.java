package org.usfirst.frc.team649.robot.util;

public class Center {
	int x, y;
	public Center(int _x, int _y){
		x = _x;
		y = _y;
	}
	
	public boolean equals(Center a){
		return a.x == this.x && a.y == this.y;
	}
	
	
}
