package copter.commands;

public class CmdSetPitchPid extends CmdSetYawRatePid
{
	public CmdSetPitchPid(boolean enabled, float kp, float ki, float kd)
	{
		super(enabled,kp,ki,kd);
		mCmdCode = 108;
	}
}
