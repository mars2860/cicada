package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

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