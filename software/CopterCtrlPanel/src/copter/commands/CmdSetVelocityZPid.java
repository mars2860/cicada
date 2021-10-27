package copter.commands;

public class CmdSetVelocityZPid extends CmdSetYawRatePid
{
	public CmdSetVelocityZPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 122;
	}
}