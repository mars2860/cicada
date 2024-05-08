package pdl.commands;

public class CmdSetVelocityY  extends AbstractCmdSetPidTarget
{
	public CmdSetVelocityY(float v)
	{
		super(126,v);
	}
}