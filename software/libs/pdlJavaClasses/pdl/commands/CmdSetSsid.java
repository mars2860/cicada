package pdl.commands;

public class CmdSetSsid extends AbstractCmdSetNetParam
{
	public CmdSetSsid(String ssid)
	{
		super(143,ssid);
	}
}
