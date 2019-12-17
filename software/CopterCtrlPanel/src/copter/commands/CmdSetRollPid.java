package copter.commands;

public class CmdSetRollPid extends CmdSetYawPid
{
	public CmdSetRollPid(boolean enabled, float kp, float ki, float kd)
	{
		super(enabled,kp,ki,kd);
		mCmdCode = 109;
	}
}