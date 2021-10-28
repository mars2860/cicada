package copter.commands;

import copter.DroneState;
import copter.DroneState.Pid;

public class CmdSetPitchPid extends CmdSetupPid
{
	public CmdSetPitchPid(Pid pid)
	{
		super(pid,108);
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mPid.settingsEquals(ds.pitchPid);
	}
}
