package pdl.commands;

public class CmdSaveDefaultCfg extends AbstractDroneCmd
{
	public CmdSaveDefaultCfg()
	{
		super(150);
	}
	
	@Override
	public byte[] getPacketData()
	{
		byte data[] = {(byte)this.getCode()};
		return data;
	}
}
