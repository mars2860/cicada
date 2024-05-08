package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

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
