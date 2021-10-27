package copter.commands;

public class CmdSetVelocityXPid extends CmdSetYawRatePid
{
	public CmdSetVelocityXPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 120;
	}
}
