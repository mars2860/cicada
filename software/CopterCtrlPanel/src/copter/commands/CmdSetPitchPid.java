package copter.commands;

public class CmdSetPitchPid extends CmdSetYawRatePid
{
	public CmdSetPitchPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 108;
	}
}
