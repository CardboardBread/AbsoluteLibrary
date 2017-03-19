package org.usfirst.frc.team4308.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private final Joystick joystick;
	private final JoystickType type;

	private final int leftAxis;
	private final int rightAxis;
	private final int turnAxis;
	private final int armAxis;

	public OI() {
		joystick = new Joystick(RobotMap.Control.driveStick);
		type = JoystickType.fromJoystick(joystick);

		switch (type) {
		case FLIGHT:
			armAxis = RobotMap.Control.Flight.throttle;
			leftAxis = RobotMap.Control.Flight.pitch;
			rightAxis = RobotMap.Control.Flight.roll;
			turnAxis = 0;
			break;
		case STANDARD:
			armAxis = RobotMap.Control.Standard.leftX;
			leftAxis = RobotMap.Control.Standard.leftY;
			rightAxis = RobotMap.Control.Standard.rightY;
			turnAxis = RobotMap.Control.Standard.rightX;
			break;
		default:
			DriverStation.reportError("Invalid number of axes on control joystick", true);
			armAxis = 0;
			leftAxis = 0;
			rightAxis = 0;
			turnAxis = 0;
			break;
		}

	}
	
	public JoystickType getJoystickType() {
		return type;
	}

	public Joystick getJoystick() {
		return joystick;
	}

	public int getLeftAxis() {
		return leftAxis;
	}

	public double getLeftValue() {
		return joystick.getRawAxis(leftAxis);
	}

	public int getMoveAxis() {
		return leftAxis;
	}
	
	public double getMoveValue() {
		return joystick.getRawAxis(leftAxis);
	}

	public int getRightAxis() {
		return rightAxis;
	}

	public double getRightValue() {
		return joystick.getRawAxis(rightAxis);
	}

	public int getRotateAxis() {
		return rightAxis;
	}
	
	public double getRotateValue() {
		return joystick.getRawAxis(rightAxis);
	}

	public int getArmAxis() {
		return armAxis;
	}

	public double getArmValue() {
		return joystick.getRawAxis(armAxis);
	}

	public int getTurnAxis() {
		return turnAxis;
	}

	public double getTurnValue() {
		return joystick.getRawAxis(turnAxis);
	}

}
