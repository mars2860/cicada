package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetYawRatePid extends CmdSetupPid
{
	public CmdSetYawRatePid(Pid pid)
	{
		super(pid, 107);
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.yawRatePid);
	}
}
