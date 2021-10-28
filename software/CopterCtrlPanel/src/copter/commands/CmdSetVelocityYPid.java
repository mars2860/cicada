package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetVelocityYPid extends CmdSetupPid
{
	public CmdSetVelocityYPid(Pid pid)
	{
		super(pid, 121);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.velocityYPid);
	}
}