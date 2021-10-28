package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetPitchRatePid extends CmdSetupPid
{
	public CmdSetPitchRatePid(Pid pid)
	{
		super(pid, 118);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.pitchRatePid);
	}
}
