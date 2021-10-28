package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

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
