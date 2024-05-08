package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetHeadingPid extends CmdSetupPid
{
	public CmdSetHeadingPid(Pid pid)
	{
		super(pid,124);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.headingPid);
	}
}