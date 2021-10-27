package copter.commands;

public class CmdSetAltPid extends CmdSetYawRatePid
{
	public CmdSetAltPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 110;
	}
}
