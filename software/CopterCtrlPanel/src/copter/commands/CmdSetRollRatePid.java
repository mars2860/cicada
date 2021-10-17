package copter.commands;

public class CmdSetRollRatePid extends CmdSetYawRatePid
{
	public CmdSetRollRatePid(boolean enabled, float kp, float ki, float kd, float maxOut)
	{
		super(enabled,kp,ki,kd,maxOut);
		mCmdCode = 119;
	}
}
