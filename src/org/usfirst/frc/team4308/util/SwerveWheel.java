package org.usfirst.frc.team4308.util;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class SwerveWheel {
	
	private SpeedController wheelPower;
	private SpeedController wheelTurn;
	
	private Encoder wheelAngle;

	public SwerveWheel(SpeedController power, SpeedController turn, int encoderA, int encoderB) {
		wheelPower = power;
		wheelTurn = turn;
		wheelAngle = new Encoder(encoderA, encoderB);
	}

}
