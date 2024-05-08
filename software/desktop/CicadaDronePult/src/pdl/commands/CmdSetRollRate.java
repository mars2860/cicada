package pdl.commands;

public class CmdSetRollRate  extends AbstractCmdSetPidTarget
{
	public CmdSetRollRate(float v)
	{
		super(135,v);
	}
}
