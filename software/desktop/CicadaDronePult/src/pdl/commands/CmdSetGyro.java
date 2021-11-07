package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.TripleAxisSensor;

public class CmdSetGyro extends CmdSetupTripleAxisSensor
{
	public CmdSetGyro(TripleAxisSensor sens)
	{
		super(sens, 103);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mSens.settingsEquals(ds.gyro);
	}
}