package org.usfirst.frc.team4308.util;

public interface Faulting {

	public boolean[] faults();

	public boolean[] stickyFaults();

	public void clearStickyFaults();

}
