package copter.commands;

public class CmdSetPitchPid extends CmdSetYawPid
{
	public CmdSetPitchPid(boolean enabled, float kp, float ki, float kd)
	{
		super(enabled,kp,ki,kd);
		mCmdCode = 108;
	}
}