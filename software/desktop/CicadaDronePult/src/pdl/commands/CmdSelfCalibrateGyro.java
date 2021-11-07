package pdl.commands;

public class CmdSelfCalibrateGyro extends AbstractCopterCmd
{
	public CmdSelfCalibrateGyro()
	{
		super(106);
	}

	@Override
	public byte[] getPacketData()
	{
		byte data[] = {(byte)this.getCode()};
		return data;
	}

}
