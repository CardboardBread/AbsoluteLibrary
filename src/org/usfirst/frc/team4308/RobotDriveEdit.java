package org.usfirst.frc.team4308;

import java.util.Objects;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;

public class RobotDriveEdit implements MotorSafety {

	public enum MotorType {
		kFrontLeft(0), kFrontRight(1), kBackLeft(2), kBackRight(3);

		private final int value;

		private MotorType(int value) {
			this.value = value;
		}
	}

	public static final double kDefaultExpirationTime = 0.1D;
	public static final double kDefaultSensitivity = 0.5D;
	public static final double kDefaultMaxOutput = 1.0D;
	protected static final int kMaxNumberOfMotors = 4;

	protected static boolean kArcadeRatioCurve_Reported = false;
	protected static boolean kTank_Reported = false;
	protected static boolean kArcadeStandard_Reported = false;
	protected static boolean kMecanumCartesian_Reported = false;
	protected static boolean kMecanumPolar_Reported = false;

	protected MotorSafetyHelper safetyHelper;

	protected CANTalon frontLeftMotor;
	protected CANTalon frontRightMotor;
	protected CANTalon backLeftMotor;
	protected CANTalon backRightMotor;

	protected final boolean leftRightDrive;

	protected double sensitivity;
	protected double maxOutput;

	public RobotDriveEdit(int leftMotorChannel, int rightMotorChannel) {
		this(new CANTalon(leftMotorChannel), new CANTalon(rightMotorChannel));
	}

	public RobotDriveEdit(int frontLeftMotor, int frontRightMotor, int backLeftMotor, int backRightMotor) {
		this(new CANTalon(frontLeftMotor), new CANTalon(frontRightMotor), new CANTalon(backLeftMotor),
				new CANTalon(backRightMotor));
	}

	public RobotDriveEdit(CANTalon leftMotor, CANTalon rightMotor) {
		this(leftMotor, rightMotor, null, null);
	}

	public RobotDriveEdit(CANTalon frontLeftMotor, CANTalon frontRightMotor, CANTalon backLeftMotor,
			CANTalon backRightMotor) {
		this.frontLeftMotor = Objects.requireNonNull(frontLeftMotor, "The front left motor cannot be null");
		this.frontRightMotor = Objects.requireNonNull(frontRightMotor, "The front right motor cannot be null");
		if (backLeftMotor == null && backRightMotor == null) {
			leftRightDrive = true;
		} else {
			leftRightDrive = false;
			this.backLeftMotor = Objects.requireNonNull(backLeftMotor, "The back left motor cannot be null");
			this.backRightMotor = Objects.requireNonNull(backRightMotor, "The back right motor cannot be null");
		}

		sensitivity = kDefaultSensitivity;
		maxOutput = kDefaultMaxOutput;
		safetyHelper = new MotorSafetyHelper(this);
		safetyHelper.setExpiration(kDefaultExpirationTime);
		safetyHelper.setSafetyEnabled(true);
	}

	/**
	 * Provide tank steering using the stored robot configuration. drive the
	 * robot using two joystick inputs. The Y-axis will be selected from each
	 * Joystick object.
	 *
	 * @param leftStick
	 *            The joystick to control the left side of the robot.
	 * @param rightStick
	 *            The joystick to control the right side of the robot.
	 */
	public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
		if (leftStick == null || rightStick == null) {
			throw new NullPointerException("Null HID provided");
		}
		tankDrive(leftStick.getY(), rightStick.getY(), true);
	}

	/**
	 * Provide tank steering using the stored robot configuration. drive the
	 * robot using two joystick inputs. The Y-axis will be selected from each
	 * Joystick object.
	 *
	 * @param leftStick
	 *            The joystick to control the left side of the robot.
	 * @param rightStick
	 *            The joystick to control the right side of the robot.
	 * @param squaredInputs
	 *            Setting this parameter to true decreases the sensitivity at
	 *            lower speeds
	 */
	public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean squaredInputs) {
		if (leftStick == null || rightStick == null) {
			throw new NullPointerException("Null HID provided");
		}
		tankDrive(leftStick.getY(), rightStick.getY(), squaredInputs);
	}

	/**
	 * Provide tank steering using the stored robot configuration. This function
	 * lets you pick the axis to be used on each Joystick object for the left
	 * and right sides of the robot.
	 *
	 * @param leftStick
	 *            The Joystick object to use for the left side of the robot.
	 * @param leftAxis
	 *            The axis to select on the left side Joystick object.
	 * @param rightStick
	 *            The Joystick object to use for the right side of the robot.
	 * @param rightAxis
	 *            The axis to select on the right side Joystick object.
	 */
	public void tankDrive(GenericHID leftStick, final int leftAxis, GenericHID rightStick, final int rightAxis) {
		if (leftStick == null || rightStick == null) {
			throw new NullPointerException("Null HID provided");
		}
		tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), true);
	}

	/**
	 * Provide tank steering using the stored robot configuration. This function
	 * lets you pick the axis to be used on each Joystick object for the left
	 * and right sides of the robot.
	 *
	 * @param leftStick
	 *            The Joystick object to use for the left side of the robot.
	 * @param leftAxis
	 *            The axis to select on the left side Joystick object.
	 * @param rightStick
	 *            The Joystick object to use for the right side of the robot.
	 * @param rightAxis
	 *            The axis to select on the right side Joystick object.
	 * @param squaredInputs
	 *            Setting this parameter to true decreases the sensitivity at
	 *            lower speeds
	 */
	public void tankDrive(GenericHID leftStick, final int leftAxis, GenericHID rightStick, final int rightAxis,
			boolean squaredInputs) {
		if (leftStick == null || rightStick == null) {
			throw new NullPointerException("Null HID provided");
		}
		tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), squaredInputs);
	}

	/**
	 * Provide tank steering using the stored robot configuration. This function
	 * lets you directly provide joystick values from any source.
	 *
	 * @param leftValue
	 *            The value of the left stick.
	 * @param rightValue
	 *            The value of the right stick.
	 * @param squaredInputs
	 *            Setting this parameter to true decreases the sensitivity at
	 *            lower speeds
	 */
	public void tankDrive(double leftValue, double rightValue, boolean squaredInputs) {

		if (!kTank_Reported) {
			HAL.report(tResourceType.kResourceType_RobotDrive, leftRightDrive ? 2 : 4, tInstances.kRobotDrive_Tank);
			kTank_Reported = true;
		}

		leftValue = limit(leftValue);
		rightValue = limit(rightValue);
		if (squaredInputs) {
			if (leftValue >= 0.0) {
				leftValue = leftValue * leftValue;
			} else {
				leftValue = -(leftValue * leftValue);
			}
			if (rightValue >= 0.0) {
				rightValue = rightValue * rightValue;
			} else {
				rightValue = -(rightValue * rightValue);
			}
		}
		setLeftRightMotorOutputs(leftValue, rightValue);
	}

	/**
	 * Provide tank steering using the stored robot configuration. This function
	 * lets you directly provide joystick values from any source.
	 *
	 * @param leftValue
	 *            The value of the left stick.
	 * @param rightValue
	 *            The value of the right stick.
	 */
	public void tankDrive(double leftValue, double rightValue) {
		tankDrive(leftValue, rightValue, true);
	}

	/**
	 * Arcade drive implements single stick driving. Given a single Joystick,
	 * the class assumes the Y axis for the move value and the X axis for the
	 * rotate value. (Should add more information here regarding the way that
	 * arcade drive works.)
	 *
	 * @param stick
	 *            The joystick to use for Arcade single-stick driving. The
	 *            Y-axis will be selected for forwards/backwards and the X-axis
	 *            will be selected for rotation rate.
	 * @param squaredInputs
	 *            If true, the sensitivity will be decreased for small values
	 */
	public void arcadeDrive(GenericHID stick, boolean squaredInputs) {
		arcadeDrive(stick.getY(), stick.getX(), squaredInputs);
	}

	/**
	 * Arcade drive implements single stick driving. Given a single Joystick,
	 * the class assumes the Y axis for the move value and the X axis for the
	 * rotate value. (Should add more information here regarding the way that
	 * arcade drive works.)
	 *
	 * @param stick
	 *            The joystick to use for Arcade single-stick driving. The
	 *            Y-axis will be selected for forwards/backwards and the X-axis
	 *            will be selected for rotation rate.
	 */
	public void arcadeDrive(GenericHID stick) {
		arcadeDrive(stick, true);
	}

	/**
	 * Arcade drive implements single stick driving. Given two joystick
	 * instances and two axis, compute the values to send to either two or four
	 * motors.
	 *
	 * @param moveStick
	 *            The Joystick object that represents the forward/backward
	 *            direction
	 * @param moveAxis
	 *            The axis on the moveStick object to use for forwards/backwards
	 *            (typically Y_AXIS)
	 * @param rotateStick
	 *            The Joystick object that represents the rotation value
	 * @param rotateAxis
	 *            The axis on the rotation object to use for the rotate
	 *            right/left (typically X_AXIS)
	 * @param squaredInputs
	 *            Setting this parameter to true decreases the sensitivity at
	 *            lower speeds
	 */
	public void arcadeDrive(GenericHID moveStick, final int moveAxis, GenericHID rotateStick, final int rotateAxis,
			boolean squaredInputs) {
		double moveValue = moveStick.getRawAxis(moveAxis);
		double rotateValue = rotateStick.getRawAxis(rotateAxis);

		arcadeDrive(moveValue, rotateValue, squaredInputs);
	}

	/**
	 * Arcade drive implements single stick driving. Given two joystick
	 * instances and two axis, compute the values to send to either two or four
	 * motors.
	 *
	 * @param moveStick
	 *            The Joystick object that represents the forward/backward
	 *            direction
	 * @param moveAxis
	 *            The axis on the moveStick object to use for forwards/backwards
	 *            (typically Y_AXIS)
	 * @param rotateStick
	 *            The Joystick object that represents the rotation value
	 * @param rotateAxis
	 *            The axis on the rotation object to use for the rotate
	 *            right/left (typically X_AXIS)
	 */
	public void arcadeDrive(GenericHID moveStick, final int moveAxis, GenericHID rotateStick, final int rotateAxis) {
		arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true);
	}

	/**
	 * Arcade drive implements single stick driving. This function lets you
	 * directly provide joystick values from any source.
	 *
	 * @param moveValue
	 *            The value to use for forwards/backwards
	 * @param rotateValue
	 *            The value to use for the rotate right/left
	 * @param squaredInputs
	 *            If set, decreases the sensitivity at low speeds
	 */
	public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
		if (!kArcadeStandard_Reported) {
			HAL.report(tResourceType.kResourceType_RobotDrive, leftRightDrive ? 2 : 4,
					tInstances.kRobotDrive_ArcadeStandard);
			kArcadeStandard_Reported = true;
		}

		double leftMotorSpeed;
		double rightMotorSpeed;

		moveValue = limit(moveValue);
		rotateValue = limit(rotateValue);

		if (squaredInputs) {
			if (moveValue >= 0.0) {
				moveValue = moveValue * moveValue;
			} else {
				moveValue = -(moveValue * moveValue);
			}
			if (rotateValue >= 0.0) {
				rotateValue = rotateValue * rotateValue;
			} else {
				rotateValue = -(rotateValue * rotateValue);
			}
		}

		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} else {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
	}

	/**
	 * Arcade drive implements single stick driving. This function lets you
	 * directly provide joystick values from any source.
	 *
	 * @param moveValue
	 *            The value to use for fowards/backwards
	 * @param rotateValue
	 *            The value to use for the rotate right/left
	 */
	public void arcadeDrive(double moveValue, double rotateValue) {
		arcadeDrive(moveValue, rotateValue, true);
	}

	/**
	 * Drive method for Mecanum wheeled robots.
	 *
	 * <p>
	 * A method for driving with Mecanum wheeled robots. There are 4 wheels on
	 * the robot, arranged so that the front and back wheels are toed in 45
	 * degrees. When looking at the wheels from the top, the roller axles should
	 * form an X across the robot.
	 *
	 * <p>
	 * This is designed to be directly driven by joystick axes.
	 *
	 * @param x
	 *            The speed that the robot should drive in the X direction.
	 *            [-1.0..1.0]
	 * @param y
	 *            The speed that the robot should drive in the Y direction. This
	 *            input is inverted to match the forward == -1.0 that joysticks
	 *            produce. [-1.0..1.0]
	 * @param rotation
	 *            The rate of rotation for the robot that is completely
	 *            independent of the translation. [-1.0..1.0]
	 * @param gyroAngle
	 *            The current angle reading from the gyro. Use this to implement
	 *            field-oriented controls.
	 */
	public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
		if (!kMecanumCartesian_Reported) {
			HAL.report(tResourceType.kResourceType_RobotDrive, leftRightDrive ? 2 : 4,
					tInstances.kRobotDrive_MecanumCartesian);
			kMecanumCartesian_Reported = true;
		}
		double xIn = x;
		double yIn = y;
		// Negate y for the joystick.
		yIn = -yIn;
		// Compensate for gyro angle.
		double[] rotated = rotateVector(xIn, yIn, gyroAngle);
		xIn = rotated[0];
		yIn = rotated[1];

		double[] wheelSpeeds = new double[kMaxNumberOfMotors];
		wheelSpeeds[MotorType.kFrontLeft.value] = xIn + yIn + rotation;
		wheelSpeeds[MotorType.kFrontRight.value] = -xIn + yIn - rotation;
		wheelSpeeds[MotorType.kBackLeft.value] = -xIn + yIn + rotation;
		wheelSpeeds[MotorType.kBackRight.value] = xIn + yIn - rotation;

		normalize(wheelSpeeds);
		frontLeftMotor.set(wheelSpeeds[MotorType.kFrontLeft.value] * maxOutput);
		frontRightMotor.set(wheelSpeeds[MotorType.kFrontRight.value] * maxOutput);
		backLeftMotor.set(wheelSpeeds[MotorType.kBackLeft.value] * maxOutput);
		backRightMotor.set(wheelSpeeds[MotorType.kBackRight.value] * maxOutput);

		if (safetyHelper != null) {
			safetyHelper.feed();
		}
	}

	/**
	 * Drive method for Mecanum wheeled robots.
	 *
	 * <p>
	 * A method for driving with Mecanum wheeled robots. There are 4 wheels on
	 * the robot, arranged so that the front and back wheels are toed in 45
	 * degrees. When looking at the wheels from the top, the roller axles should
	 * form an X across the robot.
	 *
	 * @param magnitude
	 *            The speed that the robot should drive in a given direction.
	 * @param direction
	 *            The direction the robot should drive in degrees. The direction
	 *            and maginitute are independent of the rotation rate.
	 * @param rotation
	 *            The rate of rotation for the robot that is completely
	 *            independent of the magnitude or direction. [-1.0..1.0]
	 */
	public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {
		if (!kMecanumPolar_Reported) {
			HAL.report(tResourceType.kResourceType_RobotDrive, leftRightDrive ? 2 : 4,
					tInstances.kRobotDrive_MecanumPolar);
			kMecanumPolar_Reported = true;
		}
		// Normalized for full power along the Cartesian axes.
		magnitude = limit(magnitude) * Math.sqrt(2.0);
		// The rollers are at 45 degree angles.
		double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
		double cosD = Math.cos(dirInRad);
		double sinD = Math.sin(dirInRad);

		double[] wheelSpeeds = new double[kMaxNumberOfMotors];
		wheelSpeeds[MotorType.kFrontLeft.value] = (sinD * magnitude + rotation);
		wheelSpeeds[MotorType.kFrontRight.value] = (cosD * magnitude - rotation);
		wheelSpeeds[MotorType.kBackLeft.value] = (cosD * magnitude + rotation);
		wheelSpeeds[MotorType.kBackRight.value] = (sinD * magnitude - rotation);

		normalize(wheelSpeeds);

		frontLeftMotor.set(wheelSpeeds[MotorType.kFrontLeft.value] * maxOutput);
		frontRightMotor.set(wheelSpeeds[MotorType.kFrontRight.value] * maxOutput);
		backLeftMotor.set(wheelSpeeds[MotorType.kBackLeft.value] * maxOutput);
		backRightMotor.set(wheelSpeeds[MotorType.kBackRight.value] * maxOutput);

		if (safetyHelper != null) {
			safetyHelper.feed();
		}
	}

	/**
	 * Set the speed of the right and left motors. This is used once an
	 * appropriate drive setup function is called such as twoWheelDrive(). The
	 * motors are set to "leftSpeed" and "rightSpeed" and includes flipping the
	 * direction of one side for opposing motors.
	 *
	 * @param leftOutput
	 *            The speed to send to the left side of the drive.
	 * @param rightOutput
	 *            The speed to send to the right side of the drive.
	 */
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
		if (leftRightDrive) {
			frontLeftMotor.set(limit(leftOutput) * maxOutput);
			frontRightMotor.set(limit(leftOutput) * maxOutput);
		} else {
			frontLeftMotor.set(limit(leftOutput) * maxOutput);
			frontRightMotor.set(limit(leftOutput) * maxOutput);
			backLeftMotor.set(limit(leftOutput) * maxOutput);
			backRightMotor.set(limit(leftOutput) * maxOutput);
		}
	}

	/**
	 * Limit motor values to the -1.0 to +1.0 range.
	 */
	protected static double limit(double num) {
		return num > 1.0D ? 1.0D : (num < -1.0D ? -1.0D : num);
	}

	/**
	 * Normalize all wheel speeds if the magnitude of any wheel is greater than
	 * 1.0.
	 */
	protected static void normalize(double[] wheelSpeeds) {
		double maxMagnitude = Math.abs(wheelSpeeds[0]);
		for (int i = 1; i < kMaxNumberOfMotors; i++) {
			double temp = Math.abs(wheelSpeeds[i]);
			if (maxMagnitude < temp) {
				maxMagnitude = temp;
			}
		}
		if (maxMagnitude > 1.0) {
			for (int i = 0; i < kMaxNumberOfMotors; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
			}
		}
	}

	/**
	 * Rotate a vector in Cartesian space.
	 */
	protected static double[] rotateVector(double x, double y, double angle) {
		double cosA = Math.cos(angle * (3.14159 / 180.0));
		double sinA = Math.sin(angle * (3.14159 / 180.0));
		double[] out = new double[2];
		out[0] = x * cosA - y * sinA;
		out[1] = x * sinA + y * cosA;
		return out;
	}

	/**
	 * Invert a motor direction. This is used when a motor should run in the
	 * opposite direction as the drive code would normally run it. Motors that
	 * are direct drive would be inverted, the drive code assumes that the
	 * motors are geared with one reversal.
	 *
	 * @param motor
	 *            The motor index to invert.
	 * @param isInverted
	 *            True if the motor should be inverted when operated.
	 */
	public void setInvertedMotor(MotorType motor, boolean isInverted) {
		switch (motor) {
		case kFrontLeft:
			if (frontLeftMotor == null)
				frontLeftMotor.setInverted(isInverted);
			break;
		case kFrontRight:
			if (frontRightMotor == null)
				frontRightMotor.setInverted(isInverted);
			break;
		case kBackLeft:
			if (backLeftMotor == null)
				backLeftMotor.setInverted(isInverted);
			break;
		case kBackRight:
			if (backRightMotor == null)
				backRightMotor.setInverted(isInverted);
			break;
		default:
			throw new IllegalArgumentException("Illegal motor type:" + motor);
		}
	}

	public void setSensitivity(double sensitivity) {
		this.sensitivity = sensitivity;
	}

	/**
	 * Configure the scaling factor for using AbsoluteDrive with motor
	 * controllers in a mode other than PercentVbus.
	 *
	 * @param maximum
	 *            Multiplied with the output percentage computed by the drive
	 *            functions.
	 */
	public void setMaxOutput(double maximum) {
		maxOutput = maximum;
	}

	@Override
	public void setExpiration(double expiration) {
		safetyHelper.setExpiration(expiration);
	}

	@Override
	public double getExpiration() {
		return safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		if (leftRightDrive) {
			frontLeftMotor.stopMotor();
			frontLeftMotor.stopMotor();
		} else {
			frontLeftMotor.stopMotor();
			frontRightMotor.stopMotor();
			backLeftMotor.stopMotor();
			backRightMotor.stopMotor();
		}
		safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return this.getClass().getName();
	}

}
