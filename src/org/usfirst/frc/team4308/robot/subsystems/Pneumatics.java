package org.usfirst.frc.team4308.robot.subsystems;

import org.usfirst.frc.team4308.util.Faulting;
import org.usfirst.frc.team4308.util.Loggable;
import org.usfirst.frc.team4308.util.Powered;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Sample implementation of a pneumatics system, which can log its values to the
 * dashboard. Note: The pneumatics system only controls the compressor and
 * pressure switch on the loop, separate solenoids should be controlled from
 * their respected systems.
 * 
 * @author Michael Brown
 * @see Compressor
 *
 */
public class Pneumatics extends Subsystem implements Loggable, Powered, Faulting {

	public static final int[] solenoidSupplyVoltage = { 12, 24 };

	private static final String defaultName = "Pneumatics";
	private static final int supplyVoltage = 12;

	private Compressor compressor;

	private boolean state;

	public Pneumatics(int pcmModule) {
		this(defaultName, pcmModule);
	}

	public Pneumatics(String name, int pcmModule) {
		super(name);
		compressor = new Compressor(pcmModule);
	}

	public boolean enable() {
		if (compressor == null) {
			DriverStation.reportError("Cannot start compressor, does not exist!", true);
			return false;
		} else {
			compressor.start();
			state = true;
			return true;
		}
	}

	public boolean disable() {
		if (compressor == null) {
			DriverStation.reportError("Cannot stop compressor, does not exist!", true);
			return false;
		} else {
			compressor.stop();
			state = false;
			return true;
		}
	}

	public boolean toggle() {
		if (state) {
			return disable();
		} else {
			return enable();
		}
	}

	@Override
	protected void initDefaultCommand() {
	}

	public boolean pressurized() {
		return compressor.getPressureSwitchValue();
	}

	@Override
	public double voltage() {
		return supplyVoltage;
	}

	@Override
	public double current() {
		return compressor.getCompressorCurrent();
	}

	@Override
	public double temperature() {
		return -1;
	}

	@Override
	public void log() {
		SmartDashboard.putNumber("Compressor Current", compressor.getCompressorCurrent());
		SmartDashboard.putBoolean("Pneumatics Pressurized", compressor.getPressureSwitchValue());
	}

	@Override
	public boolean[] faults() {
		return new boolean[] { compressor.getCompressorCurrentTooHighFault(),
				compressor.getCompressorNotConnectedFault(), compressor.getCompressorShortedFault() };
	}

	@Override
	public boolean[] stickyFaults() {
		return new boolean[] { compressor.getCompressorCurrentTooHighStickyFault(),
				compressor.getCompressorNotConnectedStickyFault(), compressor.getCompressorShortedStickyFault() };
	}

	@Override
	public void clearStickyFaults() {
		compressor.clearAllPCMStickyFaults();
	}

}
