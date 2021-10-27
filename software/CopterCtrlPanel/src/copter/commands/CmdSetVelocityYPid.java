package copter.commands;

public class CmdSetVelocityYPid extends CmdSetYawRatePid
{
	public CmdSetVelocityYPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 121;
	}
}