package pdl.commands;

public class CmdSetPsk extends AbstractCmdSetNetParam
{
	public CmdSetPsk(String psk)
	{
		super(144,psk);
	}
}
