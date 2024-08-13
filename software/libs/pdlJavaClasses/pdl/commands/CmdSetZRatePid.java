package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetZRatePid extends CmdSetupPid
{
	public CmdSetZRatePid(Pid pid)
	{
		super(pid, 107);
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.zRatePid);
	}
}
