package copter.commands;

import copter.DroneState;
import copter.DroneState.TripleAxisSensor;

public class CmdSetAccel extends CmdSetupTripleAxisSensor
{
	public CmdSetAccel(TripleAxisSensor sens)
	{
		super(sens,102);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mSens.settingsEquals(ds.accel);
	}
}
