package pdl.commands;

public class CmdSetVelocityZ extends AbstractCmdSetPidTarget
{
	public CmdSetVelocityZ(float v)
	{
		super(123,v);
	}
}