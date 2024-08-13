package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

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
