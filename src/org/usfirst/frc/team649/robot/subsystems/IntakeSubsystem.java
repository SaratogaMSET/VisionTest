package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeSubsystem extends Subsystem {

	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	public static double STOP_SPEED = 0;
	Victor[] rollers;
	public DoubleSolenoid leftSolenoid;
	public DoubleSolenoid rightSolenoid;

	public IntakeSubsystem() {
		rollers = new Victor[3];
		leftSolenoid = new DoubleSolenoid(RobotMap.Intake.LEFT_SOLENOID_PORTS[0],
				RobotMap.Intake.LEFT_SOLENOID_PORTS[1],RobotMap.Intake.LEFT_SOLENOID_PORTS[2]);
		rightSolenoid = new DoubleSolenoid(RobotMap.Intake.RIGHT_SOLENOID_PORTS[0],
				RobotMap.Intake.RIGHT_SOLENOID_PORTS[1],RobotMap.Intake.RIGHT_SOLENOID_PORTS[2]);
		for (int i = 0; i < rollers.length; i++) {
			rollers[i] = new Victor(RobotMap.Intake.MOTOR_PORTS[i]);
		}
	}

	public void setFwdRolSpd(double speed) {
		rollers[2].set(speed);
	}

	public void setCenteringModuleSpeed(double speed) {
		rollers[0].set(speed);
		rollers[1].set(-speed);
	}
	//true = up, false = down
	public void setSolenoids(boolean set) {
		if (set){
			leftSolenoid.set(DoubleSolenoid.Value.kForward);
			rightSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		else{
			leftSolenoid.set(DoubleSolenoid.Value.kReverse);
			rightSolenoid.set(DoubleSolenoid.Value.kReverse);
		}

	}

	protected void initDefaultCommand() {

	}
}