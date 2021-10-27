package copter.commands;

public class CmdSetOpticalFlowYPid extends CmdSetYawRatePid
{
	public CmdSetOpticalFlowYPid(boolean enabled, float kp, float ki, float kd, float maxOut, float maxErrSum)
	{
		super(enabled,kp,ki,kd,maxOut,maxErrSum);
		mCmdCode = 121;
	}
}