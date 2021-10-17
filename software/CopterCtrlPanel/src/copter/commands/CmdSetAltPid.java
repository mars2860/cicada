package copter.commands;

public class CmdSetAltPid extends CmdSetYawRatePid
{
	public CmdSetAltPid(boolean enabled, float kp, float ki, float kd, float maxOut)
	{
		super(enabled,kp,ki,kd,maxOut);
		mCmdCode = 110;
	}
}
