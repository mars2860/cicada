package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetRollPid extends CmdSetupPid
{
	public CmdSetRollPid(Pid pid)
	{
		super(pid,109);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.rollPid);
	}
}
