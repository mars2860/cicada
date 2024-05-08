package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetPosNorthPid extends CmdSetupPid
{
	public CmdSetPosNorthPid(Pid pid)
	{
		super(pid,132);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.posNorthPid);
	}
}