package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetXRatePid extends CmdSetupPid
{
	public CmdSetXRatePid(Pid pid)
	{
		super(pid,119);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.xRatePid);
	}
}
