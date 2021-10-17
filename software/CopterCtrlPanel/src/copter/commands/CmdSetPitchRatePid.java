package copter.commands;

public class CmdSetPitchRatePid extends CmdSetYawRatePid
{
	public CmdSetPitchRatePid(boolean enabled, float kp, float ki, float kd, float maxOut)
	{
		super(enabled,kp,ki,kd,maxOut);
		mCmdCode = 118;
	}
}
