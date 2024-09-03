package pdl.commands;

public class CmdSetZRate extends AbstractCmdSetPidTarget
{
	public CmdSetZRate(float v)
	{
		super(157,v);
	}
}
