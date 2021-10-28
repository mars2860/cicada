package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetRollRatePid extends CmdSetupPid
{
	public CmdSetRollRatePid(Pid pid)
	{
		super(pid,119);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.rollRatePid);
	}
}
