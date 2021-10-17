package copter.commands;

public class CmdSetRollPid extends CmdSetYawRatePid
{
	public CmdSetRollPid(boolean enabled, float kp, float ki, float kd, float maxOut)
	{
		super(enabled,kp,ki,kd,maxOut);
		mCmdCode = 109;
	}
}
