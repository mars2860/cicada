package pdl.commands;

public class CmdSetPitchRate  extends AbstractCmdSetPidTarget
{
	public CmdSetPitchRate(float v)
	{
		super(134,v);
	}
}
