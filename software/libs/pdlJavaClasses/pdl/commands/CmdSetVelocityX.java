package pdl.commands;

public class CmdSetVelocityX  extends AbstractCmdSetPidTarget
{
	public CmdSetVelocityX(float v)
	{
		super(125,v);
	}
}