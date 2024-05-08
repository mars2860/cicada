package pdl.commands;

public class CmdLoadDefaultCfg extends AbstractDroneCmd
{
	public CmdLoadDefaultCfg()
	{
		super(151);
	}
	
	@Override
	public byte[] getPacketData()
	{
		byte data[] = {(byte)this.getCode()};
		return data;
	}
}
