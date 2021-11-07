package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.Pid;

public class CmdSetVelocityZPid extends CmdSetupPid
{
	public CmdSetVelocityZPid(Pid pid)
	{
		super(pid, 122);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.velocityZPid);
	}
}