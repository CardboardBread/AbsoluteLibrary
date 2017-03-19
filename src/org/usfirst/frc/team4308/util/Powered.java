package org.usfirst.frc.team4308.util;

/**
 * Utility interface allowing the monitoring of power consumption and power
 * output efficiency (temperature)
 * 
 * @author Michael Brown
 *
 */
public interface Powered {

	public double voltage();

	public double current();

	public double temperature();

	public default double wattage() {
		return voltage() * current();
	}

}
