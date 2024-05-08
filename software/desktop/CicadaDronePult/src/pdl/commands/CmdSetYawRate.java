package pdl.commands;

public class CmdSetYawRate  extends AbstractCmdSetPidTarget
{
	public CmdSetYawRate(float v)
	{
		super(130,v);
	}
}
