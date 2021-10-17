package copter.commands;

public class CmdSetPitchPid extends CmdSetYawRatePid
{
	public CmdSetPitchPid(boolean enabled, float kp, float ki, float kd, float maxOut)
	{
		super(enabled,kp,ki,kd,maxOut);
		mCmdCode = 108;
	}
}
