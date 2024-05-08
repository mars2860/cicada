package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetPosEastPid extends CmdSetupPid
{
	public CmdSetPosEastPid(Pid pid)
	{
		super(pid,133);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.posEastPid);
	}
}