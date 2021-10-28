package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetAltPid extends CmdSetupPid
{
	public CmdSetAltPid(Pid pid)
	{
		super(pid, 110);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.altPid);
	}
}
