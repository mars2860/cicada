package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.TripleAxisSensor;

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
