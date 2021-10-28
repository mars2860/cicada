package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetVelocityXPid extends CmdSetupPid
{
	public CmdSetVelocityXPid(Pid pid)
	{
		super(pid, 120);
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.velocityXPid);
	}
}
