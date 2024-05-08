package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetYRatePid extends CmdSetupPid
{
	public CmdSetYRatePid(Pid pid)
	{
		super(pid, 118);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.yRatePid);
	}
}
