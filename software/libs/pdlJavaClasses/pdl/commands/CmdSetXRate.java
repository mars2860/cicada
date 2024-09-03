package pdl.commands;

public class CmdSetXRate extends AbstractCmdSetPidTarget
{
	public CmdSetXRate(float v)
	{
		super(158,v);
	}
}