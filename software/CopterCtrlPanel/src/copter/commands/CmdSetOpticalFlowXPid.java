package copter.commands;

public class CmdSetOpticalFlowXPid extends CmdSetYawRatePid
{
	public CmdSetOpticalFlowXPid(boolean enabled, float kp, float ki, float kd, float maxOut)
	{
		super(enabled,kp,ki,kd,maxOut);
		mCmdCode = 120;
	}
}
