package pdl.commands;

public class CmdSetYRate extends AbstractCmdSetPidTarget
{
	public CmdSetYRate(float v)
	{
		super(159,v);
	}
}