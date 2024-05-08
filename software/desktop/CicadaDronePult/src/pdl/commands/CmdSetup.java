package pdl.commands;

import pdl.DroneState;

public abstract class CmdSetup extends AbstractDroneCmd
{
	public CmdSetup(int code)
	{
		super(code);
	}
	
	/// Check current settings with settings setup by command
	public abstract boolean settingsEquals(DroneState ds);
}
