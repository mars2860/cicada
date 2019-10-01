package copter.commands;

public class CmdSetAltPid extends CmdSetYawPid
{
	public CmdSetAltPid(boolean enabled, float kp, float ki, float kd)
	{
		super(enabled,kp,ki,kd);
		mCmdCode = 110;
	}
}
