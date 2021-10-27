package copter.commands;

public class CmdSetRollPid extends CmdSetYawRatePid
{
	public CmdSetRollPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 109;
	}
}
