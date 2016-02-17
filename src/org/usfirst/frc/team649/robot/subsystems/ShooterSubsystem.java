package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {

	private static final double ENCODER_DISTANCE_PER_PULSE = 0;
	public Victor leftMotor;
	public Victor rightMotor;
	public DoubleSolenoid punch;
	public Counter counter;
	
	public ShooterSubsystem() {
		super("shooter subsystem");
		leftMotor = new Victor(RobotMap.ShooterSubsystem.MOTOR_PORTS[0]);
		rightMotor = new Victor(RobotMap.ShooterSubsystem.MOTOR_PORTS[1]);
		punch = new DoubleSolenoid(
				RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[0],
				RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[1]);
		
		counter.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
		counter.setReverseDirection(false);
	}

	public void runPunch(Value punchPower) {
		punch.set(punchPower);
	}

	public void resetCounter() {
		counter.reset();
	}

	public boolean reachedLimit() {
		return counter.get() > 0;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
