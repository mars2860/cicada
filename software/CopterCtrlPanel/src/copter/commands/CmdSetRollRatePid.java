package copter.commands;

public class CmdSetRollRatePid extends CmdSetYawRatePid
{
	public CmdSetRollRatePid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 119;
	}
}
