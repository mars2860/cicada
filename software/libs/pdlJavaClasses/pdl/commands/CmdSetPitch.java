package pdl.commands;

public class CmdSetPitch  extends AbstractCmdSetPidTarget
{
	public CmdSetPitch(float v)
	{
		super(128,v);
	}
}
